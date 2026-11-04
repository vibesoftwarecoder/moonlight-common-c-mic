#include "Limelight-internal.h"

// Uncomment to test 3 byte Annex B start sequences with GFE
//#define FORCE_3_BYTE_START_SEQUENCES

#define DR_CLEANUP -1000

#define CONSECUTIVE_DROP_LIMIT 120

// Everything the depacketizer remembers about one video stream. A connection has one per stream.
struct _VIDEO_DEPACKETIZER {
    int streamIndex;

    PLENTRY nalChainHead;
    PLENTRY nalChainTail;
    int nalChainDataLength;

    unsigned int nextFrameNumber;
    unsigned int startFrameNumber;
    bool waitingForNextSuccessfulFrame;
    bool waitingForIdrFrame;
    bool waitingForRefInvalFrame;
    unsigned int lastPacketInStream;
    bool decodingFrame;
    int frameType;
    uint16_t lastPacketPayloadLength;
    bool strictIdrFrameWait;
    uint64_t syntheticPtsBaseUs;
    uint16_t frameHostProcessingLatency;
    uint64_t firstPacketReceiveTimeUs;
    uint64_t firstPacketPresentationTime;
    uint32_t firstPacketRtpTimestamp;
    bool dropStatePending;
    bool idrFrameProcessed;

    unsigned int consecutiveFrameDrops;

    LINKED_BLOCKING_QUEUE decodeUnitQueue;
};

static VIDEO_DEPACKETIZER videoDepacketizers[MAX_VIDEO_STREAMS];

PVIDEO_DEPACKETIZER getVideoDepacketizer(int streamIndex) {
    LC_ASSERT(streamIndex >= 0 && streamIndex < MAX_VIDEO_STREAMS);
    return &videoDepacketizers[streamIndex];
}

// Only the first stream may talk to the host on the control stream: the host applies those requests to its
// first stream, so a second stream would corrupt it. Stream-scoped requests come later.
static void requestIdrFrameForStream(PVIDEO_DEPACKETIZER d) {
    if (d->streamIndex == 0) {
        LiRequestIdrFrame();
    }
}

static void reportFrameLossForStream(PVIDEO_DEPACKETIZER d, uint32_t startFrame, uint32_t endFrame) {
    if (d->streamIndex == 0) {
        connectionDetectedFrameLoss(startFrame, endFrame);
    }
}

typedef struct _BUFFER_DESC {
    char* data;
    unsigned int offset;
    unsigned int length;
} BUFFER_DESC, *PBUFFER_DESC;

typedef struct _LENTRY_INTERNAL {
    LENTRY entry;
    void* allocPtr;
} LENTRY_INTERNAL, *PLENTRY_INTERNAL;

#define H264_NAL_TYPE(x) ((x) & 0x1F)
#define HEVC_NAL_TYPE(x) (((x) & 0x7E) >> 1)

#define H264_NAL_TYPE_SEI 6
#define H264_NAL_TYPE_SPS 7
#define H264_NAL_TYPE_PPS 8
#define H264_NAL_TYPE_AUD 9
#define H264_NAL_TYPE_FILLER 12
#define HEVC_NAL_TYPE_VPS 32
#define HEVC_NAL_TYPE_SPS 33
#define HEVC_NAL_TYPE_PPS 34
#define HEVC_NAL_TYPE_AUD 35
#define HEVC_NAL_TYPE_FILLER 38
#define HEVC_NAL_TYPE_SEI 39

// Init
void initializeVideoDepacketizer(PVIDEO_DEPACKETIZER d, int streamIndex, int pktSize) {
    d->streamIndex = streamIndex;
    LbqInitializeLinkedBlockingQueue(&d->decodeUnitQueue, 15);

    d->nextFrameNumber = 1;
    d->startFrameNumber = 0;
    d->waitingForNextSuccessfulFrame = false;
    d->waitingForIdrFrame = true;
    d->waitingForRefInvalFrame = false;
    d->lastPacketInStream = UINT32_MAX;
    d->decodingFrame = false;
    d->syntheticPtsBaseUs = 0;
    d->frameHostProcessingLatency = 0;
    d->firstPacketReceiveTimeUs = 0;
    d->firstPacketPresentationTime = 0;
    d->firstPacketRtpTimestamp = 0;
    d->lastPacketPayloadLength = 0;
    d->dropStatePending = false;
    d->idrFrameProcessed = false;
    d->strictIdrFrameWait = !isReferenceFrameInvalidationEnabled();
}

// Free the NAL chain
static void cleanupFrameState(PVIDEO_DEPACKETIZER d) {
    PLENTRY_INTERNAL lastEntry;

    while (d->nalChainHead != NULL) {
        lastEntry = (PLENTRY_INTERNAL)d->nalChainHead;
        d->nalChainHead = lastEntry->entry.next;
        free(lastEntry->allocPtr);
    }

    d->nalChainTail = NULL;

    d->nalChainDataLength = 0;
}

// Cleanup frame state and set that we're waiting for an IDR Frame
static void dropFrameState(PVIDEO_DEPACKETIZER d) {
    // This may only be called at frame boundaries
    LC_ASSERT(!d->decodingFrame);

    // We're dropping frame state now
    d->dropStatePending = false;

    if (d->strictIdrFrameWait || !d->idrFrameProcessed || d->waitingForIdrFrame || (d->nalChainHead && d->frameType == FRAME_TYPE_IDR)) {
        // We'll need an IDR frame now if we're in non-RFI mode, if we've never
        // received an IDR frame, if we explicitly need an IDR frame, or if we
        // just dropped a partially processed IDR frame.
        d->waitingForIdrFrame = true;
    }
    else {
        d->waitingForRefInvalFrame = true;
    }

    // Count the number of consecutive frames dropped
    d->consecutiveFrameDrops++;

    // If we reach our limit, immediately request an IDR frame and reset
    if (d->consecutiveFrameDrops == CONSECUTIVE_DROP_LIMIT) {
        Limelog("Reached consecutive drop limit\n");

        // Restart the count
        d->consecutiveFrameDrops = 0;

        // Request an IDR frame
        d->waitingForIdrFrame = true;
        requestIdrFrameForStream(d);
    }

    cleanupFrameState(d);
}

// Cleanup the list of decode units
static void freeDecodeUnitList(PLINKED_BLOCKING_QUEUE_ENTRY entry) {
    PLINKED_BLOCKING_QUEUE_ENTRY nextEntry;

    while (entry != NULL) {
        nextEntry = entry->flink;

        // Complete this with a failure status
        LiCompleteVideoFrame(entry->data, DR_CLEANUP);

        entry = nextEntry;
    }
}

void stopVideoDepacketizer(PVIDEO_DEPACKETIZER d) {
    LbqSignalQueueShutdown(&d->decodeUnitQueue);
}

// Cleanup video depacketizer and free malloced memory
void destroyVideoDepacketizer(PVIDEO_DEPACKETIZER d) {
    freeDecodeUnitList(LbqDestroyLinkedBlockingQueue(&d->decodeUnitQueue));
    cleanupFrameState(d);
}

// NB: This function also ensures an additional byte for the NALU type exists after the start sequence
static bool getAnnexBStartSequence(PBUFFER_DESC current, PBUFFER_DESC startSeq) {
    // We must not get called for other codecs
    LC_ASSERT(NegotiatedVideoFormat & (VIDEO_FORMAT_MASK_H264 | VIDEO_FORMAT_MASK_H265));

    if (current->length <= 3) {
        return false;
    }

    if (current->data[current->offset] == 0 &&
        current->data[current->offset + 1] == 0) {
        if (current->data[current->offset + 2] == 0) {
            if (current->length > 4 && current->data[current->offset + 3] == 1) {
                // Frame start
                if (startSeq != NULL) {
                    startSeq->data = current->data;
                    startSeq->offset = current->offset;
                    startSeq->length = 4;
                }
                return true;
            }
        }
        else if (current->data[current->offset + 2] == 1) {
            // NAL start
            if (startSeq != NULL) {
                startSeq->data = current->data;
                startSeq->offset = current->offset;
                startSeq->length = 3;
            }
            return true;
        }
    }

    return false;
}

void validateDecodeUnitForPlayback(PDECODE_UNIT decodeUnit) {
    // Frames must always have at least one buffer
    LC_ASSERT(decodeUnit->bufferList != NULL);
    LC_ASSERT(decodeUnit->fullLength != 0);

    // Validate the buffers in the frame
    if (decodeUnit->frameType == FRAME_TYPE_IDR) {
        // IDR frames always start with codec configuration data
        if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_H264) {
            // H.264 IDR frames should have an SPS, PPS, then picture data
            LC_ASSERT_VT(decodeUnit->bufferList->bufferType == BUFFER_TYPE_SPS);
            LC_ASSERT_VT(decodeUnit->bufferList->next != NULL);
            LC_ASSERT_VT(decodeUnit->bufferList->next->bufferType == BUFFER_TYPE_PPS);
            LC_ASSERT_VT(decodeUnit->bufferList->next->next != NULL);
        }
        else if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_H265) {
            // HEVC IDR frames should have an VPS, SPS, PPS, then picture data
            LC_ASSERT_VT(decodeUnit->bufferList->bufferType == BUFFER_TYPE_VPS);
            LC_ASSERT_VT(decodeUnit->bufferList->next != NULL);
            LC_ASSERT_VT(decodeUnit->bufferList->next->bufferType == BUFFER_TYPE_SPS);
            LC_ASSERT_VT(decodeUnit->bufferList->next->next != NULL);
            LC_ASSERT_VT(decodeUnit->bufferList->next->next->bufferType == BUFFER_TYPE_PPS);
            LC_ASSERT_VT(decodeUnit->bufferList->next->next->next != NULL);
        }
        else if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_AV1) {
            // We don't parse the AV1 bitstream
            LC_ASSERT_VT(decodeUnit->bufferList->bufferType == BUFFER_TYPE_PICDATA);
        }
        else {
            LC_ASSERT(false);
        }
    }
    else {
        LC_ASSERT(decodeUnit->frameType == FRAME_TYPE_PFRAME);

        // P frames always start with picture data
        LC_ASSERT(decodeUnit->bufferList->bufferType == BUFFER_TYPE_PICDATA);

        // We must not dequeue a P frame before an IDR frame has been successfully processed
        LC_ASSERT(d->idrFrameProcessed);
    }
}

bool LiWaitForNextVideoFrame(VIDEO_FRAME_HANDLE* frameHandle, PDECODE_UNIT* decodeUnit) {
    PVIDEO_DEPACKETIZER d = getVideoDepacketizer(0);
    PQUEUED_DECODE_UNIT qdu;

    int err = LbqWaitForQueueElement(&d->decodeUnitQueue, (void**)&qdu);
    if (err != LBQ_SUCCESS) {
        return false;
    }

    validateDecodeUnitForPlayback(&qdu->decodeUnit);

    *frameHandle = qdu;
    *decodeUnit = &qdu->decodeUnit;
    return true;
}

bool LiPollNextVideoFrame(VIDEO_FRAME_HANDLE* frameHandle, PDECODE_UNIT* decodeUnit) {
    PVIDEO_DEPACKETIZER d = getVideoDepacketizer(0);
    PQUEUED_DECODE_UNIT qdu;

    int err = LbqPollQueueElement(&d->decodeUnitQueue, (void**)&qdu);
    if (err != LBQ_SUCCESS) {
        return false;
    }

    validateDecodeUnitForPlayback(&qdu->decodeUnit);

    *frameHandle = qdu;
    *decodeUnit = &qdu->decodeUnit;
    return true;
}

bool LiPeekNextVideoFrame(PDECODE_UNIT* decodeUnit) {
    PVIDEO_DEPACKETIZER d = getVideoDepacketizer(0);
    PQUEUED_DECODE_UNIT qdu;

    int err = LbqPeekQueueElement(&d->decodeUnitQueue, (void**)&qdu);
    if (err != LBQ_SUCCESS) {
        return false;
    }

    validateDecodeUnitForPlayback(&qdu->decodeUnit);

    *decodeUnit = &qdu->decodeUnit;
    return true;
}

void LiWakeWaitForVideoFrame(void) {
    PVIDEO_DEPACKETIZER d = getVideoDepacketizer(0);
    LbqSignalQueueUserWake(&d->decodeUnitQueue);
}

// Cleanup a decode unit by freeing the buffer chain and the holder
void LiCompleteVideoFrame(VIDEO_FRAME_HANDLE handle, int drStatus) {
    PQUEUED_DECODE_UNIT qdu = handle;
    PVIDEO_DEPACKETIZER d = qdu->owner;
    PLENTRY_INTERNAL lastEntry;

    if (drStatus == DR_NEED_IDR) {
        Limelog("Requesting IDR frame on behalf of DR\n");
        requestDecoderRefresh(d);
    }
    else if (drStatus == DR_OK && qdu->decodeUnit.frameType == FRAME_TYPE_IDR) {
        // Remember that the IDR frame was processed. We can now use
        // reference frame invalidation.
        d->idrFrameProcessed = true;
    }

    while (qdu->decodeUnit.bufferList != NULL) {
        lastEntry = (PLENTRY_INTERNAL)qdu->decodeUnit.bufferList;
        qdu->decodeUnit.bufferList = lastEntry->entry.next;
        free(lastEntry->allocPtr);
    }

    // We will have stack-allocated entries iff we have a direct-submit decoder
    if ((VideoCallbacks.capabilities & CAPABILITY_DIRECT_SUBMIT) == 0) {
        free(qdu);
    }
}

static bool isSeqReferenceFrameStart(PBUFFER_DESC buffer) {
    BUFFER_DESC startSeq;

    if (!getAnnexBStartSequence(buffer, &startSeq)) {
        return false;
    }

    if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_H264) {
        return H264_NAL_TYPE(startSeq.data[startSeq.offset + startSeq.length]) == 5;
    }
    else if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_H265) {
        switch (HEVC_NAL_TYPE(startSeq.data[startSeq.offset + startSeq.length])) {
            case 16:
            case 17:
            case 18:
            case 19:
            case 20:
            case 21:
                return true;

            default:
                return false;
        }
    }
    else {
        LC_ASSERT(false);
        return false;
    }
}

static bool isAccessUnitDelimiter(PBUFFER_DESC buffer) {
    BUFFER_DESC startSeq;

    if (!getAnnexBStartSequence(buffer, &startSeq)) {
        return false;
    }

    if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_H264) {
        return H264_NAL_TYPE(startSeq.data[startSeq.offset + startSeq.length]) == H264_NAL_TYPE_AUD;
    }
    else if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_H265) {
        return HEVC_NAL_TYPE(startSeq.data[startSeq.offset + startSeq.length]) == HEVC_NAL_TYPE_AUD;
    }
    else {
        LC_ASSERT(false);
        return false;
    }
}

static bool isSeiNal(PBUFFER_DESC buffer) {
    BUFFER_DESC startSeq;

    if (!getAnnexBStartSequence(buffer, &startSeq)) {
        return false;
    }

    if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_H264) {
        return H264_NAL_TYPE(startSeq.data[startSeq.offset + startSeq.length]) == H264_NAL_TYPE_SEI;
    }
    else if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_H265) {
        return HEVC_NAL_TYPE(startSeq.data[startSeq.offset + startSeq.length]) == HEVC_NAL_TYPE_SEI;
    }
    else {
        LC_ASSERT(false);
        return false;
    }
}

#ifdef LC_DEBUG
static bool isFillerDataNal(PBUFFER_DESC buffer) {
    BUFFER_DESC startSeq;

    if (!getAnnexBStartSequence(buffer, &startSeq)) {
        return false;
    }

    if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_H264) {
        return H264_NAL_TYPE(startSeq.data[startSeq.offset + startSeq.length]) == H264_NAL_TYPE_FILLER;
    }
    else if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_H265) {
        return HEVC_NAL_TYPE(startSeq.data[startSeq.offset + startSeq.length]) == HEVC_NAL_TYPE_FILLER;
    }
    else {
        LC_ASSERT(false);
        return false;
    }
}
#endif

static bool isPictureParameterSetNal(PBUFFER_DESC buffer) {
    BUFFER_DESC startSeq;

    if (!getAnnexBStartSequence(buffer, &startSeq)) {
        return false;
    }

    if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_H264) {
        return H264_NAL_TYPE(startSeq.data[startSeq.offset + startSeq.length]) == H264_NAL_TYPE_PPS;
    }
    else if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_H265) {
        return HEVC_NAL_TYPE(startSeq.data[startSeq.offset + startSeq.length]) == HEVC_NAL_TYPE_PPS;
    }
    else {
        LC_ASSERT(false);
        return false;
    }
}

// Advance the buffer descriptor to the start of the next NAL or end of buffer
static void skipToNextNalOrEnd(PBUFFER_DESC buffer) {
    BUFFER_DESC startSeq;

    // If we're starting on a NAL boundary, skip to the next one
    if (getAnnexBStartSequence(buffer, &startSeq)) {
        buffer->offset += startSeq.length;
        buffer->length -= startSeq.length;
    }

    // Loop until we find an Annex B start sequence (3 or 4 byte)
    while (!getAnnexBStartSequence(buffer, NULL)) {
        if (buffer->length == 0) {
            // Reached the end of the buffer
            return;
        }

        buffer->offset++;
        buffer->length--;
    }
}

// Advance the buffer descriptor to the start of the next NAL
static void skipToNextNal(PBUFFER_DESC buffer) {
    skipToNextNalOrEnd(buffer);

    // If we skipped all the data, something has gone horribly wrong
    LC_ASSERT(buffer->length > 0);
}

static bool isIdrFrameStart(PBUFFER_DESC buffer) {
    BUFFER_DESC startSeq;

    if (!getAnnexBStartSequence(buffer, &startSeq)) {
        return false;
    }

    if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_H264) {
        return H264_NAL_TYPE(startSeq.data[startSeq.offset + startSeq.length]) == H264_NAL_TYPE_SPS;
    }
    else if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_H265) {
        return HEVC_NAL_TYPE(startSeq.data[startSeq.offset + startSeq.length]) == HEVC_NAL_TYPE_VPS;
    }
    else {
        LC_ASSERT(false);
        return false;
    }
}

// Reassemble the frame with the given frame number
static void reassembleFrame(PVIDEO_DEPACKETIZER d, int frameNumber, bool frameIsLTR) {
    if (d->nalChainHead != NULL) {
        QUEUED_DECODE_UNIT qduDS;
        PQUEUED_DECODE_UNIT qdu;

        // Use a stack allocation if we won't be queuing this
        if ((VideoCallbacks.capabilities & CAPABILITY_DIRECT_SUBMIT) == 0) {
            qdu = (PQUEUED_DECODE_UNIT)malloc(sizeof(*qdu));
        }
        else {
            qdu = &qduDS;
        }

        if (qdu != NULL) {
            qdu->owner = d;
            qdu->decodeUnit.bufferList = d->nalChainHead;
            qdu->decodeUnit.fullLength = d->nalChainDataLength;
            qdu->decodeUnit.frameType = d->frameType;
            qdu->decodeUnit.frameNumber = frameNumber;
            qdu->decodeUnit.frameHostProcessingLatency = d->frameHostProcessingLatency;
            qdu->decodeUnit.receiveTimeUs = d->firstPacketReceiveTimeUs;
            qdu->decodeUnit.presentationTimeUs = d->firstPacketPresentationTime;
            qdu->decodeUnit.rtpTimestamp = d->firstPacketRtpTimestamp;
            qdu->decodeUnit.enqueueTimeUs = PltGetMicroseconds();

            // These might be wrong for a few frames during a transition between SDR and HDR,
            // but the effects shouldn't very noticable since that's an infrequent operation.
            //
            // If we start sending this state in the frame header, we can make it 100% accurate.
            qdu->decodeUnit.hdrActive = LiGetCurrentHostDisplayHdrMode();
            qdu->decodeUnit.colorspace = (uint8_t)(qdu->decodeUnit.hdrActive ? COLORSPACE_REC_2020 : StreamConfig.colorSpace);

            // Invoke the key frame callback if needed
            if (d->nalChainHead->bufferType != BUFFER_TYPE_PICDATA || qdu->decodeUnit.frameType == FRAME_TYPE_IDR) {
                qdu->decodeUnit.frameType = FRAME_TYPE_IDR;
                notifyKeyFrameReceived();
            }
            else {
                qdu->decodeUnit.frameType = FRAME_TYPE_PFRAME;
            }

            d->nalChainHead = d->nalChainTail = NULL;
            d->nalChainDataLength = 0;

            if ((VideoCallbacks.capabilities & CAPABILITY_DIRECT_SUBMIT) == 0) {
                if (LbqOfferQueueItem(&d->decodeUnitQueue, qdu, &qdu->entry) == LBQ_BOUND_EXCEEDED) {
                    Limelog("Video decode unit queue overflow\n");

                    // RFI recovery is not supported here
                    d->waitingForIdrFrame = true;

                    // Clear NAL state for the frame that we failed to enqueue
                    d->nalChainHead = qdu->decodeUnit.bufferList;
                    d->nalChainDataLength = qdu->decodeUnit.fullLength;
                    dropFrameState(d);

                    // Free the DU we were going to queue
                    free(qdu);

                    // Free all frames in the decode unit queue
                    freeDecodeUnitList(LbqFlushQueueItems(&d->decodeUnitQueue));

                    // Request an IDR frame to recover
                    requestIdrFrameForStream(d);
                    return;
                }
            }
            else {
                // Submit the frame to the decoder
                validateDecodeUnitForPlayback(&qdu->decodeUnit);
                LiCompleteVideoFrame(qdu, VideoCallbacks.submitDecodeUnit(&qdu->decodeUnit));
            }

            // Notify the control connection
            if (d->streamIndex == 0) {
                connectionReceivedCompleteFrame(frameNumber, frameIsLTR);
            }

            // Clear frame drops
            d->consecutiveFrameDrops = 0;

            // Move the start of our (potential) RFI window to the next frame
            d->startFrameNumber = d->nextFrameNumber;
        }
    }
}

static int getBufferFlags(char* data, int length) {
    BUFFER_DESC buffer;
    BUFFER_DESC candidate;

    // We only parse H.264 and HEVC bitstreams
    if (!(NegotiatedVideoFormat & (VIDEO_FORMAT_MASK_H264 | VIDEO_FORMAT_MASK_H265))) {
        return BUFFER_TYPE_PICDATA;
    }

    buffer.data = data;
    buffer.length = (unsigned int)length;
    buffer.offset = 0;

    if (!getAnnexBStartSequence(&buffer, &candidate)) {
        return BUFFER_TYPE_PICDATA;
    }

    if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_H264) {
        switch (H264_NAL_TYPE(candidate.data[candidate.offset + candidate.length])) {
        case H264_NAL_TYPE_SPS:
            return BUFFER_TYPE_SPS;

        case H264_NAL_TYPE_PPS:
            return BUFFER_TYPE_PPS;

        default:
            return BUFFER_TYPE_PICDATA;
        }
    }
    else if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_H265) {
        switch (HEVC_NAL_TYPE(candidate.data[candidate.offset + candidate.length])) {
            case HEVC_NAL_TYPE_SPS:
                return BUFFER_TYPE_SPS;

            case HEVC_NAL_TYPE_PPS:
                return BUFFER_TYPE_PPS;

            case HEVC_NAL_TYPE_VPS:
                return BUFFER_TYPE_VPS;

            default:
                return BUFFER_TYPE_PICDATA;
        }
    }
    else {
        LC_ASSERT(false);
        return BUFFER_TYPE_PICDATA;
    }
}

// As an optimization, we can cast the existing packet buffer to a PLENTRY and avoid
// a malloc() and a memcpy() of the packet data.
static void queueFragment(PVIDEO_DEPACKETIZER d, PLENTRY_INTERNAL* existingEntry, char* data, int offset, int length) {
    PLENTRY_INTERNAL entry;

    if (existingEntry == NULL || *existingEntry == NULL) {
        entry = (PLENTRY_INTERNAL)malloc(sizeof(*entry) + length);
    }
    else {
        entry = *existingEntry;
    }

    if (entry != NULL) {
        entry->entry.next = NULL;
        entry->entry.length = length;

        // If we had to allocate a new entry, we must copy the data. If not,
        // the data already resides within the LENTRY allocation.
        if (existingEntry == NULL || *existingEntry == NULL) {
            entry->allocPtr = entry;

            entry->entry.data = (char*)(entry + 1);
            memcpy(entry->entry.data, &data[offset], entry->entry.length);
        }
        else {
            entry->entry.data = &data[offset];

            // The caller should have already set this up for us
            LC_ASSERT(entry->allocPtr != NULL);

            // We now own the packet buffer and will manage freeing it
            *existingEntry = NULL;
        }

        entry->entry.bufferType = getBufferFlags(entry->entry.data, entry->entry.length);

        d->nalChainDataLength += entry->entry.length;

        if (d->nalChainTail == NULL) {
            LC_ASSERT(d->nalChainHead == NULL);
            d->nalChainHead = d->nalChainTail = (PLENTRY)entry;
        }
        else {
            LC_ASSERT(d->nalChainHead != NULL);
            d->nalChainTail->next = (PLENTRY)entry;
            d->nalChainTail = d->nalChainTail->next;
        }
    }
}

// Process an RTP Payload using the slow path that handles multiple NALUs per packet
static void processAvcHevcRtpPayloadSlow(PVIDEO_DEPACKETIZER d, PBUFFER_DESC currentPos, PLENTRY_INTERNAL* existingEntry) {
    // We should not have any NALUs when processing the first packet in an IDR frame
    LC_ASSERT(d->nalChainHead == NULL);
    LC_ASSERT(d->nalChainTail == NULL);

    while (currentPos->length != 0) {
        // Skip through any padding bytes
        if (!getAnnexBStartSequence(currentPos, NULL)) {
            skipToNextNal(currentPos);
        }

        // Skip any prepended AUD or SEI NALUs. We may have padding between
        // these on IDR frames, so the check in processRtpPayload() is not
        // completely sufficient to handle that case.
        while (isAccessUnitDelimiter(currentPos) || isSeiNal(currentPos)) {
            skipToNextNal(currentPos);
        }

        int start = currentPos->offset;
        bool containsPicData = false;

#ifdef FORCE_3_BYTE_START_SEQUENCES
        start++;
#endif

        if (isSeqReferenceFrameStart(currentPos)) {
            // No longer waiting for an IDR frame
            d->waitingForIdrFrame = false;
            d->waitingForRefInvalFrame = false;

            // Cancel any pending IDR frame request
            d->waitingForNextSuccessfulFrame = false;

            // Use the cached LENTRY for this NALU since it will be
            // the bulk of the data in this packet.
            containsPicData = true;

            // This is an IDR frame
            d->frameType = FRAME_TYPE_IDR;
        }

        // Move to the next NALU
        skipToNextNalOrEnd(currentPos);

        // If this is the picture data, we expect it to extend to the end of the packet
        if (containsPicData) {
            while (currentPos->length != 0) {
                // Any NALUs we encounter on the way to the end of the packet must be
                // reference frame slices or filler data.
                LC_ASSERT_VT(isSeqReferenceFrameStart(currentPos) || isFillerDataNal(currentPos));
                skipToNextNalOrEnd(currentPos);
            }
        }

        // To minimize copies, we'll allocate for SPS, PPS, and VPS to allow
        // us to reuse the packet buffer for the picture data in the I-frame.
        queueFragment(d, containsPicData ? existingEntry : NULL,
                      currentPos->data, start, currentPos->offset - start);
    }
}

// Dumps the decode unit queue and ensures the next frame submitted to the decoder will be
// an IDR frame
void requestDecoderRefresh(PVIDEO_DEPACKETIZER d) {
    // Wait for the next IDR frame
    d->waitingForIdrFrame = true;

    // Flush the decode unit queue
    freeDecodeUnitList(LbqFlushQueueItems(&d->decodeUnitQueue));

    // Request the receive thread drop its state
    // on the next call. We can't do it here because
    // it may be trying to queue DUs and we'll nuke
    // the state out from under it.
    d->dropStatePending = true;

    // Request the IDR frame
    requestIdrFrameForStream(d);
}

// Return 1 if packet is the first one in the frame
static bool isFirstPacket(uint8_t flags, uint8_t fecBlockNumber) {
    // Clear the picture data flag
    flags &= ~FLAG_CONTAINS_PIC_DATA;

    // Check if it's just the start or both start and end of a frame
    return (flags == (FLAG_SOF | FLAG_EOF) || flags == FLAG_SOF) && fecBlockNumber == 0;
}

// Process an RTP Payload
// The caller will free *existingEntry unless we NULL it
static void processRtpPayload(PVIDEO_DEPACKETIZER d, PNV_VIDEO_PACKET videoPacket, int length,
                       uint64_t receiveTimeUs, uint64_t presentationTimeUs, uint32_t rtpTimestamp,
                       PLENTRY_INTERNAL* existingEntry) {
    BUFFER_DESC currentPos;
    uint32_t frameIndex;
    uint8_t flags;
    uint8_t extraFlags;
    bool firstPacket, lastPacket;
    uint32_t streamPacketIndex;
    uint8_t fecCurrentBlockNumber;
    uint8_t fecLastBlockNumber;

    // Mask the top 8 bits from the SPI
    videoPacket->streamPacketIndex >>= 8;
    videoPacket->streamPacketIndex &= 0xFFFFFF;

    currentPos.data = (char*)(videoPacket + 1);
    currentPos.offset = 0;
    currentPos.length = length - sizeof(*videoPacket);

    fecCurrentBlockNumber = (videoPacket->multiFecBlocks >> 4) & 0x3;
    fecLastBlockNumber = (videoPacket->multiFecBlocks >> 6) & 0x3;
    frameIndex = videoPacket->frameIndex;
    flags = videoPacket->flags;
    extraFlags = videoPacket->extraFlags;
    firstPacket = isFirstPacket(flags, fecCurrentBlockNumber);
    lastPacket = (flags & FLAG_EOF) && fecCurrentBlockNumber == fecLastBlockNumber;

    LC_ASSERT_VT((flags & ~(FLAG_SOF | FLAG_EOF | FLAG_CONTAINS_PIC_DATA)) == 0);

    streamPacketIndex = videoPacket->streamPacketIndex;

    // Drop packets from a previously corrupt frame
    if (isBefore32(frameIndex, d->nextFrameNumber)) {
        return;
    }

    // The FEC queue can sometimes recover corrupt frames (see comments in RtpFecQueue).
    // It almost always detects them before they get to us, but in case it doesn't
    // the streamPacketIndex not matching correctly should find nearly all of the rest.
    if (isBefore24(streamPacketIndex, U24(d->lastPacketInStream + 1)) ||
            (!(flags & FLAG_SOF) && streamPacketIndex != U24(d->lastPacketInStream + 1))) {
        Limelog("Depacketizer detected corrupt frame: %d", frameIndex);
        d->decodingFrame = false;
        d->nextFrameNumber = frameIndex + 1;
        dropFrameState(d);
        if (d->waitingForIdrFrame) {
            requestIdrFrameForStream(d);
        }
        else {
            reportFrameLossForStream(d, d->startFrameNumber, frameIndex);
        }
        return;
    }

    // Verify that we didn't receive an incomplete frame
    LC_ASSERT(firstPacket ^ d->decodingFrame);

    // Check sequencing of this frame to ensure we didn't
    // miss one in between
    if (firstPacket) {
        // Make sure this is the next consecutive frame
        if (isBefore32(d->nextFrameNumber, frameIndex)) {
            if (d->nextFrameNumber + 1 == frameIndex) {
                Limelog("Network dropped 1 frame (frame %d)\n", frameIndex - 1);
            }
            else {
                Limelog("Network dropped %d frames (frames %d to %d)\n",
                        frameIndex - d->nextFrameNumber,
                        d->nextFrameNumber,
                        frameIndex - 1);
            }

            d->nextFrameNumber = frameIndex;

            // Wait until next complete frame
            d->waitingForNextSuccessfulFrame = true;
            dropFrameState(d);
        }
        else {
            LC_ASSERT(d->nextFrameNumber == frameIndex);
        }

        // We're now decoding a frame
        d->decodingFrame = true;
        d->frameType = FRAME_TYPE_PFRAME;
        d->firstPacketReceiveTimeUs = receiveTimeUs;

        // Some versions of Sunshine don't send a valid PTS, so we will
        // synthesize one using the receive time as the time base.
        if (!d->syntheticPtsBaseUs) {
            d->syntheticPtsBaseUs = receiveTimeUs;
        }

        if (!presentationTimeUs && frameIndex > 0) {
            d->firstPacketPresentationTime = receiveTimeUs - d->syntheticPtsBaseUs;
        }
        else {
            d->firstPacketPresentationTime = presentationTimeUs;
        }

        d->firstPacketRtpTimestamp = rtpTimestamp;
    }

    d->lastPacketInStream = streamPacketIndex;

    // If this is the first packet, skip the frame header (if one exists)
    uint32_t frameHeaderSize;
    LC_ASSERT_VT(currentPos.length > 0);
    if (firstPacket && currentPos.length > 0) {
        // Parse the frame type from the header
        LC_ASSERT_VT(currentPos.length >= 4);
        if (APP_VERSION_AT_LEAST(7, 1, 350) && currentPos.length >= 4) {
            switch (currentPos.data[currentPos.offset + 3]) {
            case 1: // Normal P-frame
                break;
            case 2: // IDR frame
                // For other codecs, we trust the frame header rather than parsing the bitstream
                // to determine if a given frame is an IDR frame.
                if (!(NegotiatedVideoFormat & (VIDEO_FORMAT_MASK_H264 | VIDEO_FORMAT_MASK_H265))) {
                    d->waitingForIdrFrame = false;
                    d->waitingForNextSuccessfulFrame = false;
                    d->frameType = FRAME_TYPE_IDR;
                }
                // Fall-through
            case 4: // Intra-refresh
            case 5: // P-frame with reference frames invalidated
                if (d->waitingForRefInvalFrame) {
                    Limelog("Next post-invalidation frame is: %d (%s-frame)\n",
                            frameIndex,
                            currentPos.data[currentPos.offset + 3] == 5 ? "P" : "I");
                    d->waitingForRefInvalFrame = false;
                    d->waitingForNextSuccessfulFrame = false;
                }
                break;
            case 104: // Sunshine hardcoded header
                break;
            default:
                Limelog("Unrecognized frame type: %d", currentPos.data[currentPos.offset + 3]);
                LC_ASSERT_VT(false);
                break;
            }
        }
        else {
            // Hope for the best with older servers
            if (d->waitingForRefInvalFrame) {
                reportFrameLossForStream(d, d->startFrameNumber, frameIndex - 1);
                d->waitingForRefInvalFrame = false;
                d->waitingForNextSuccessfulFrame = false;
            }
        }

        // Sunshine can provide host processing latency of the frame
        LC_ASSERT_VT(currentPos.length >= 3);
        if (IS_SUNSHINE() && currentPos.length >= 3) {
            BYTE_BUFFER bb;
            BbInitializeWrappedBuffer(&bb, currentPos.data, currentPos.offset + 1, 2, BYTE_ORDER_LITTLE);
            BbGet16(&bb, &d->frameHostProcessingLatency);
        }

        // Codecs like H.264 and HEVC handle the FEC trailing zero padding just fine, but other
        // codecs need the exact length encoded separately.
        LC_ASSERT_VT(currentPos.length >= 6);
        if (!(NegotiatedVideoFormat & (VIDEO_FORMAT_MASK_H264 | VIDEO_FORMAT_MASK_H265)) && currentPos.length >= 6) {
            BYTE_BUFFER bb;
            BbInitializeWrappedBuffer(&bb, currentPos.data, currentPos.offset + 4, 2, BYTE_ORDER_LITTLE);
            BbGet16(&bb, &d->lastPacketPayloadLength);
        }

        if (APP_VERSION_AT_LEAST(7, 1, 450)) {
            // >= 7.1.450 uses 2 different header lengths based on the first byte:
            // 0x01 indicates an 8 byte header
            // 0x81 indicates a 44 byte header
            if (currentPos.data[0] == 0x01) {
                frameHeaderSize = 8;
            }
            else {
                LC_ASSERT_VT(currentPos.data[0] == (char)0x81);
                frameHeaderSize = 44;
            }
        }
        else if (APP_VERSION_AT_LEAST(7, 1, 446)) {
            // [7.1.446, 7.1.450) uses 2 different header lengths based on the first byte:
            // 0x01 indicates an 8 byte header
            // 0x81 indicates a 41 byte header
            if (currentPos.data[0] == 0x01) {
                frameHeaderSize = 8;
            }
            else {
                LC_ASSERT_VT(currentPos.data[0] == (char)0x81);
                frameHeaderSize = 41;
            }
        }
        else if (APP_VERSION_AT_LEAST(7, 1, 415)) {
            // [7.1.415, 7.1.446) uses 2 different header lengths based on the first byte:
            // 0x01 indicates an 8 byte header
            // 0x81 indicates a 24 byte header
            if (currentPos.data[0] == 0x01) {
                frameHeaderSize = 8;
            }
            else {
                LC_ASSERT_VT(currentPos.data[0] == (char)0x81);
                frameHeaderSize = 24;
            }
        }
        else if (APP_VERSION_AT_LEAST(7, 1, 350)) {
            // [7.1.350, 7.1.415) should use the 8 byte header again
            frameHeaderSize = 8;
        }
        else if (APP_VERSION_AT_LEAST(7, 1, 320)) {
            // [7.1.320, 7.1.350) should use the 12 byte frame header
            frameHeaderSize = 12;
        }
        else if (APP_VERSION_AT_LEAST(5, 0, 0)) {
            // [5.x, 7.1.320) should use the 8 byte header
            frameHeaderSize = 8;
        }
        else {
            // Other versions don't have a frame header at all
            frameHeaderSize = 0;
        }

        LC_ASSERT_VT(currentPos.length >= frameHeaderSize);
        if (currentPos.length >= frameHeaderSize) {
            // Skip past the frame header
            currentPos.offset += frameHeaderSize;
            currentPos.length -= frameHeaderSize;
        }

        // We only parse H.264 and HEVC at the NALU level
        if (NegotiatedVideoFormat & (VIDEO_FORMAT_MASK_H264 | VIDEO_FORMAT_MASK_H265)) {
            // The Annex B NALU start prefix must be next
            if (!getAnnexBStartSequence(&currentPos, NULL)) {
                // If we aren't starting on a start prefix, something went wrong.
                LC_ASSERT_VT(false);

                // For release builds, we will try to recover by searching for one.
                // This mimics the way most decoders handle this situation.
                skipToNextNal(&currentPos);
            }

            // If an AUD NAL is prepended to this frame data, remove it.
            // Other parts of this code are not prepared to deal with a
            // NAL of that type, so stripping it is the easiest option.
            if (isAccessUnitDelimiter(&currentPos)) {
                skipToNextNal(&currentPos);
            }

            // There may be one or more SEI NAL units prepended to the
            // frame data *after* the (optional) AUD.
            while (isSeiNal(&currentPos)) {
                skipToNextNal(&currentPos);
            }
        }
    }
    else {
        // There is no frame header on later packets
        frameHeaderSize = 0;
    }

    if (NegotiatedVideoFormat & (VIDEO_FORMAT_MASK_H264 | VIDEO_FORMAT_MASK_H265)) {
        if (firstPacket && isIdrFrameStart(&currentPos)) {
            // SPS and PPS prefix is padded between NALs, so we must decode it with the slow path
            processAvcHevcRtpPayloadSlow(d, &currentPos, existingEntry);
        }
        else {
            // Intel's H.264 Media Foundation encoder prepends a PPS to each P-frame.
            // Skip it to avoid confusing clients.
            if (firstPacket && isPictureParameterSetNal(&currentPos)) {
                skipToNextNal(&currentPos);
            }

#ifdef FORCE_3_BYTE_START_SEQUENCES
            if (firstPacket) {
                currentPos.offset++;
                currentPos.length--;
            }
#endif

            queueFragment(d, existingEntry, currentPos.data, currentPos.offset, currentPos.length);
        }
    }
    else {
        // We fixup the length of the last packet for other codecs since they may not be tolerant
        // of trailing zero padding like H.264/HEVC Annex B bitstream parsers are.
        if (lastPacket) {
            // The payload length includes the frame header, so it cannot be smaller than that
            LC_ASSERT_VT(d->lastPacketPayloadLength > frameHeaderSize);

            // The payload length cannot be smaller than the actual received payload
            // NB: currentPos.length is already adjusted to exclude the frameHeaderSize from above
            LC_ASSERT_VT(d->lastPacketPayloadLength - frameHeaderSize <= currentPos.length);

            // If the payload length is valid, truncate the packet. If not, discard this frame.
            if (d->lastPacketPayloadLength > frameHeaderSize && d->lastPacketPayloadLength - frameHeaderSize <= currentPos.length) {
                currentPos.length = d->lastPacketPayloadLength - frameHeaderSize;
            }
            else {
                if (d->lastPacketPayloadLength <= frameHeaderSize) {
                    Limelog("Invalid last payload length for header on frame %u: %u <= %u",
                            frameIndex, d->lastPacketPayloadLength, frameHeaderSize);
                }
                else {
                    Limelog("Invalid last payload length for packet size on frame %u: %u > %u",
                            frameIndex, d->lastPacketPayloadLength - frameHeaderSize, currentPos.length);
                }

                // Skip to the next frame and tell the host we lost this one
                d->decodingFrame = false;
                d->nextFrameNumber = frameIndex + 1;
                dropFrameState(d);
                if (d->waitingForIdrFrame) {
                    requestIdrFrameForStream(d);
                }
                else {
                    reportFrameLossForStream(d, d->startFrameNumber, frameIndex);
                }

                return;
            }
        }

        // Other codecs are just passed through as is.
        queueFragment(d, existingEntry, currentPos.data, currentPos.offset, currentPos.length);
    }

    if (lastPacket) {
        // Move on to the next frame
        d->decodingFrame = false;
        d->nextFrameNumber = frameIndex + 1;

        // If we can't submit this frame due to a discontinuity in the bitstream,
        // inform the host (if needed) and drop the data.
        if (d->waitingForIdrFrame || d->waitingForRefInvalFrame) {
            // IDR wait takes priority over RFI wait (and an IDR frame will satisfy both)
            if (d->waitingForIdrFrame) {
                Limelog("Waiting for IDR frame\n");

                // We wait for the first fully received frame after a loss to approximate
                // detection of the recovery of the network. Requesting an IDR frame while
                // the network is unstable will just contribute to congestion collapse.
                if (d->waitingForNextSuccessfulFrame) {
                    requestIdrFrameForStream(d);
                }
            }
            else {
                // If we need an RFI frame first, then drop this frame
                // and update the reference frame invalidation window.
                Limelog("Waiting for RFI frame\n");
                reportFrameLossForStream(d, d->startFrameNumber, frameIndex);
            }

            d->waitingForNextSuccessfulFrame = false;
            dropFrameState(d);
            return;
        }

        LC_ASSERT(!d->waitingForNextSuccessfulFrame);

        // Carry out any pending state drops. We can't just do this
        // arbitrarily in the middle of processing a frame because
        // may cause the depacketizer state to become corrupted. For
        // example, if we drop state after the first packet, the
        // depacketizer will next try to process a non-SOF packet,
        // and cause it to assert.
        if (d->dropStatePending) {
            if (d->nalChainHead && d->frameType == FRAME_TYPE_IDR) {
                // Don't drop the frame state if this frame is an IDR frame itself,
                // otherwise we'll lose this IDR frame without another in flight
                // and have to wait until we hit our consecutive drop limit to
                // request a new one (potentially several seconds).
                d->dropStatePending = false;
            }
            else {
                dropFrameState(d);
                return;
            }
        }

        reassembleFrame(d, frameIndex, extraFlags & NV_VIDEO_PACKET_EXTRA_FLAG_LTR_FRAME);
    }
}

// Called by the video RTP FEC queue to notify us of a lost frame
// if it determines the frame to be unrecoverable. This lets us
// avoid having to wait until the next received frame to determine
// that we lost a frame and submit an RFI request.
void notifyFrameLost(PVIDEO_DEPACKETIZER d, unsigned int frameNumber, bool speculative) {
    // We may not invalidate frames that we've already received
    LC_ASSERT(frameNumber >= d->startFrameNumber);

    // Drop state and determine if we need an IDR frame or if RFI is okay
    dropFrameState(d);

    // If dropFrameState() determined that RFI was usable, issue it now
    if (!d->waitingForIdrFrame) {
        LC_ASSERT(d->waitingForRefInvalFrame);

        if (speculative) {
            Limelog("Sending speculative RFI request for predicted loss of frame %d\n", frameNumber);
        }
        else {
            Limelog("Sending RFI request for unrecoverable frame %d\n", frameNumber);
        }

        // Advance the frame number since we won't be expecting this one anymore
        d->nextFrameNumber = frameNumber + 1;

        // Notify the host that we lost this one
        reportFrameLossForStream(d, d->startFrameNumber, frameNumber);
    }
}

// Add an RTP Packet to the queue
void queueRtpPacket(PVIDEO_DEPACKETIZER d, PRTPV_QUEUE_ENTRY queueEntryPtr) {
    int dataOffset;
    RTPV_QUEUE_ENTRY queueEntry = *queueEntryPtr;

    LC_ASSERT(!queueEntry.isParity);
    LC_ASSERT(queueEntry.receiveTimeUs != 0);

    dataOffset = sizeof(*queueEntry.packet);
    if (queueEntry.packet->header & FLAG_EXTENSION) {
        dataOffset += 4; // 2 additional fields
    }

    // The packet length was validated by the RtpVideoQueue
    LC_ASSERT(queueEntry.length >= dataOffset + (int)sizeof(NV_VIDEO_PACKET));

    // Reuse the memory reserved for the RTPFEC_QUEUE_ENTRY to store the LENTRY_INTERNAL
    // now that we're in the depacketizer. We saved a copy of the real FEC queue entry
    // on the stack here so we can safely modify this memory in place.
    LC_ASSERT(sizeof(LENTRY_INTERNAL) <= sizeof(RTPV_QUEUE_ENTRY));
    PLENTRY_INTERNAL existingEntry = (PLENTRY_INTERNAL)queueEntryPtr;
    existingEntry->allocPtr = queueEntry.packet;

    processRtpPayload(d, (PNV_VIDEO_PACKET)(((char*)queueEntry.packet) + dataOffset),
                      queueEntry.length - dataOffset,
                      queueEntry.receiveTimeUs,
                      queueEntry.presentationTimeUs,
                      queueEntry.rtpTimestamp,
                      &existingEntry);

    if (existingEntry != NULL) {
        // processRtpPayload didn't want this packet, so just free it
        free(existingEntry->allocPtr);
    }
}

int LiGetPendingVideoFrames(void) {
    PVIDEO_DEPACKETIZER d = getVideoDepacketizer(0);
    return LbqGetItemCount(&d->decodeUnitQueue);
}
