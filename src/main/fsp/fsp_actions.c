#include "fsp_actions.h"
#include <string.h>

void fspActionsInit(fspActionScheduler_t *s) { memset(s, 0, sizeof(*s)); }

static fspActionResult_e reject(fspActionScheduler_t *s, fspActionResult_e reason)
{
    s->status.result = reason;
    s->status.rejectedCount++;
    if (reason == FSP_ACTION_LATE) s->status.lateCount++;
    if (reason == FSP_ACTION_FULL) s->status.fullCount++;
    return reason;
}

fspActionResult_e fspActionsEnqueue(fspActionScheduler_t *s, const fspActionPacket_t *a)
{
    s->status.receivedSequence = a->sequence;
    if (a->flags || a->reserved || (a->mode != FSP_ACTION_RATES && a->mode != FSP_ACTION_MOTORS))
        return reject(s, FSP_ACTION_INVALID);
    for (unsigned i = 0; i < 4; i++) {
        const int low = (a->mode == FSP_ACTION_RATES && i < 3) ? -20000 : 0;
        const int high = (a->mode == FSP_ACTION_RATES && i < 3) ? 20000 : 10000;
        if (a->values[i] < low || a->values[i] > high) return reject(s, FSP_ACTION_INVALID);
    }
    const int32_t ahead = (int32_t)(a->applyTick - s->status.pidTick);
    if (ahead <= 0) return reject(s, FSP_ACTION_LATE);
    if (ahead > (int32_t)FSP_ACTION_MAX_AHEAD_TICKS || (int32_t)(a->sourceTick - s->status.pidTick) > 0 ||
        (int32_t)(a->applyTick - a->sourceTick) <= 0)
        return reject(s, FSP_ACTION_HORIZON);
    if ((s->haveSequence && (int32_t)(a->sequence - s->lastSequence) <= 0) ||
        (s->count && (int32_t)(a->applyTick - s->queue[(s->head + s->count - 1) % FSP_ACTION_QUEUE_CAPACITY].applyTick) <= 0))
        return reject(s, FSP_ACTION_ORDER);
    if (s->count == FSP_ACTION_QUEUE_CAPACITY) return reject(s, FSP_ACTION_FULL);
    s->queue[(s->head + s->count) % FSP_ACTION_QUEUE_CAPACITY] = *a;
    s->count++;
    s->lastSequence = a->sequence;
    s->haveSequence = true;
    s->status.queueDepth = s->count;
    s->status.result = FSP_ACTION_OK;
    return FSP_ACTION_OK;
}

bool fspActionsOutputActive(const fspActionScheduler_t *s)
{
    return s->authority && s->haveActive;
}

void fspActionsTick(fspActionScheduler_t *s, uint32_t nowUs, uint32_t periodUs, bool authority)
{
    s->status.pidTick++;
    s->status.header.timestamp = nowUs;
    s->status.pidPeriodUs = periodUs;
    if (authority != s->authority) {
        s->head = s->count = 0;
        s->haveActive = s->haveSequence = false;
        s->status.result = FSP_ACTION_AUTHORITY_CHANGED;
        s->status.mode = 0;
        memset(s->status.values, 0, sizeof(s->status.values));
    }
    s->authority = authority;
    while (s->count) {
        const fspActionPacket_t *a = &s->queue[s->head];
        const int32_t due = (int32_t)(s->status.pidTick - a->applyTick);
        if (due < 0) break;
        if (due == 0) {
            s->active = *a;
            s->haveActive = true;
            s->status.consumedSequence = a->sequence;
            s->status.requestedTick = a->applyTick;
            s->status.consumedTick = s->status.pidTick;
            s->status.consumedTimeUs = nowUs;
            s->status.mode = a->mode;
            memcpy(s->status.values, a->values, sizeof(a->values));
        } else reject(s, FSP_ACTION_LATE);
        s->head = (s->head + 1) % FSP_ACTION_QUEUE_CAPACITY;
        s->count--;
    }
    s->status.timedOut = s->haveActive && (uint32_t)(nowUs - s->status.consumedTimeUs) >= FSP_ACTION_TIMEOUT_US;
    s->status.queueDepth = s->count;
    s->status.authority = authority;
    s->status.outputActive = fspActionsOutputActive(s);
}
