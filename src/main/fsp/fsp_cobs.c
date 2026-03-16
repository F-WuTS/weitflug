#include "fsp/fsp_cobs.h"

static void fspDecoderReset(fspCobsDecoder_t *decoder)
{
    decoder->bufferIndex = 0;
    decoder->blockCode = 0xFF;
    decoder->blockRemaining = 0;
}

void fspCobsDecoderInit(fspCobsDecoder_t *decoder, uint8_t *buffer, size_t bufferSize)
{
    decoder->buffer = buffer;
    decoder->bufferSize = bufferSize;
    fspDecoderReset(decoder);
}

fspCobsDecoderResult_e fspCobsDecoderPush(fspCobsDecoder_t *decoder, uint8_t byte, size_t *decodedLength)
{
    if (decoder->blockRemaining == 0) {
        if (byte == 0) {
            // End of packet
            *decodedLength = decoder->bufferIndex;
            fspDecoderReset(decoder);
            return FSP_COBS_DECODER_DONE;
        }

        // Insert a zero, if this is not a continuation block
        if (decoder->blockCode != 0xFF) {
            if (decoder->bufferIndex >= decoder->bufferSize) {
                fspDecoderReset(decoder);
                return FSP_COBS_DECODER_ERROR_BUFFER_OVERFLOW;
            }
            decoder->buffer[decoder->bufferIndex++] = 0;
        }

        decoder->blockCode = decoder->blockRemaining = byte;
    }
    else {
        if (byte == 0) {
            // Zero bytes are not allowed in the middle of a block
            fspDecoderReset(decoder);
            return FSP_COBS_DECODER_ERROR_INVALID_INPUT;
        }

        // Decode a data byte
        if (decoder->bufferIndex >= decoder->bufferSize) {
            fspDecoderReset(decoder);
            return FSP_COBS_DECODER_ERROR_BUFFER_OVERFLOW;
        }
        decoder->buffer[decoder->bufferIndex++] = byte;
    }

    decoder->blockRemaining--;

    return FSP_COBS_DECODER_IN_PROGRESS;
}

bool fspCobsEncode(const uint8_t *input, size_t inputLength, uint8_t *output, size_t outputSize, size_t *encodedLength)
{
    size_t inputIndex = 0;
    size_t outputIndex = 1;
    size_t blockStartIndex = 0;
    uint8_t blockCode = 1;

    while (inputIndex < inputLength) {
        if (outputIndex >= outputSize) {
            return false; // Output buffer overflow
        }

        if (input[inputIndex] != 0) {
            output[outputIndex++] = input[inputIndex];
            blockCode++;
        }

        if (input[inputIndex] == 0 || blockCode == 0xFF) {
            // End of block
            output[blockStartIndex] = blockCode;
            blockCode = 1;
            blockStartIndex = outputIndex;
            if (input[inputIndex] == 0 || inputIndex < inputLength - 1) {
                // Incrementing is skipped if the 0xFF code reaches exactly the end of
                // the input
                outputIndex++;
            }
        }

        inputIndex++;
    }

    if (outputIndex >= outputSize) {
        return false; // Output buffer overflow
    }

    output[blockStartIndex] = blockCode;
    output[outputIndex++] = 0; // Packet delimiter
    *encodedLength = outputIndex;

    return true;
}
