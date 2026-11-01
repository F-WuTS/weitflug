#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "fsp_frame.h"

#define FSP_ACTION_QUEUE_CAPACITY 4U

// Authority levels passed to fspActionsTick
#define FSP_AUTHORITY_RC (1U << 0)      // MSP override selected, radio healthy: RC actions allowed
#define FSP_AUTHORITY_CONTROL (1U << 1) // armed, Acro, supported: rate and motor actions allowed (needs RC too)

typedef struct {
    fspPacketAction_t queue[FSP_ACTION_QUEUE_CAPACITY];
    uint8_t head, count;
    uint8_t authority; // FSP_AUTHORITY_* bits
    bool haveActive, activated, timedOut, haveTick;
    fspPacketAction_t active;
    uint32_t activeTimeUs;
    uint32_t pidTick;
    uint32_t lastTickUs;
    uint32_t timeoutUs;
    uint32_t maxAheadTicks;
    uint32_t periodNs;
    fspPacketStats_t stats;
} fspActionScheduler_t;

void fspActionsInit(fspActionScheduler_t *s, uint32_t timeoutUs, uint32_t maxAheadTicks, uint32_t nominalPeriodUs);
fspActionResult_e fspActionsEnqueue(fspActionScheduler_t *s, const fspPacketAction_t *action);
void fspActionsTick(fspActionScheduler_t *s, uint32_t nowUs, uint8_t authority);
// The active action may drive its outputs (authority sufficient for its mode).
bool fspActionsOutputActive(const fspActionScheduler_t *s);
void fspActionsFrameState(const fspActionScheduler_t *s, fspSensorFrame_t *frame);
