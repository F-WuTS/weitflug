#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint8_t *buffer;
    size_t bufferSize;
    size_t bufferIndex;
    uint8_t blockCode;
    uint8_t blockRemaining;
} fspCobsDecoder_t;

typedef enum {
    FSP_COBS_DECODER_DONE,
    FSP_COBS_DECODER_IN_PROGRESS,
    FSP_COBS_DECODER_ERROR_BUFFER_OVERFLOW,
    FSP_COBS_DECODER_ERROR_INVALID_INPUT
} fspCobsDecoderResult_e;

void fspCobsDecoderInit(fspCobsDecoder_t *decoder, uint8_t *buffer, size_t bufferSize);
fspCobsDecoderResult_e fspCobsDecoderPush(fspCobsDecoder_t *decoder, uint8_t byte, size_t *decodedLength);
bool fspCobsEncode(const uint8_t *input, size_t inputLength, uint8_t *output, size_t outputSize, size_t *encodedLength);
