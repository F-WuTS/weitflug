#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "fsp_frame.h"

#define FSP_ACTION_QUEUE_CAPACITY 4U

// Authority levels passed to fspActionsTick; reported as fspStatusAuthority_e
#define FSP_AUTHORITY_RC FSP_STATUS_AUTHORITY_RC           // MSP override selected, radio healthy
#define FSP_AUTHORITY_CONTROL FSP_STATUS_AUTHORITY_CONTROL // armed, Acro, supported; rate and motor actions need both bits

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
// RC actions skip the scheduler: validate only, applyTick is ignored.
fspActionResult_e fspActionsCheckRc(fspActionScheduler_t *s, const fspPacketAction_t *action);
// Rate and motor actions only; RC actions are rejected as invalid.
fspActionResult_e fspActionsEnqueue(fspActionScheduler_t *s, const fspPacketAction_t *action);
void fspActionsTick(fspActionScheduler_t *s, uint32_t nowUs, uint8_t authority);
// The active action may drive its outputs (authority sufficient for its mode).
bool fspActionsOutputActive(const fspActionScheduler_t *s);
// Per-capture action state: flags with the mode, age of the action in effect and queue depth.
void fspActionsFrameState(const fspActionScheduler_t *s, fspSensorFrame_t *frame);
// Exact tick and mode of the action in effect and the authority bits, for the status packet.
void fspActionsStatus(const fspActionScheduler_t *s, fspPacketStatus_t *status);
