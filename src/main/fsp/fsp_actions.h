#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "fsp_frame.h"

typedef struct {
    fspActionPacket_t queue[FSP_ACTION_QUEUE_CAPACITY];
    uint8_t head, count;
    bool haveSequence, haveActive, authority;
    uint32_t lastSequence;
    fspActionPacket_t active;
    fspActionStatus_t status;
} fspActionScheduler_t;

void fspActionsInit(fspActionScheduler_t *s);
fspActionResult_e fspActionsEnqueue(fspActionScheduler_t *s, const fspActionPacket_t *action);
void fspActionsTick(fspActionScheduler_t *s, uint32_t nowUs, uint32_t periodUs, bool authority);
bool fspActionsOutputActive(const fspActionScheduler_t *s);
