#include "fsp_actions.h"
#include <string.h>

// Skip and period tracking work in ns; EMA weight 1/64 of each tick
#define FSP_PERIOD_EMA_SHIFT 6

void fspActionsInit(fspActionScheduler_t *s, uint32_t timeoutUs, uint32_t maxAheadTicks, uint32_t nominalPeriodUs)
{
    memset(s, 0, sizeof(*s));
    s->timeoutUs = timeoutUs;
    s->maxAheadTicks = maxAheadTicks;
    s->periodNs = nominalPeriodUs * 1000U;
    s->stats.pidPeriodNs = s->periodNs;
}

static fspActionResult_e reject(fspActionScheduler_t *s, fspActionResult_e reason)
{
    if (reason == FSP_ACTION_LATE) {
        s->stats.lateCount++;
    } else if (reason == FSP_ACTION_FULL) {
        s->stats.fullCount++;
    } else {
        s->stats.rejectedCount++;
    }
    return reason;
}

static bool valuesValid(const fspPacketAction_t *a)
{
    for (unsigned i = 0; i < 4; i++) {
        int low, high;
        switch (a->mode) {
        case FSP_ACTION_MODE_RC:
            low = 1000; high = 2000;
            break;
        case FSP_ACTION_MODE_RATES:
            low = i < 3 ? -20000 : 0; high = i < 3 ? 20000 : 10000;
            break;
        case FSP_ACTION_MODE_MOTORS:
            low = 0; high = 10000;
            break;
        default:
            return false;
        }
        if (a->values[i] < low || a->values[i] > high) {
            return false;
        }
    }
    return true;
}

static bool actionValid(const fspPacketAction_t *a)
{
    return !a->reserved[0] && !a->reserved[1] && !a->reserved[2] && valuesValid(a);
}

fspActionResult_e fspActionsCheckRc(fspActionScheduler_t *s, const fspPacketAction_t *a)
{
    if (a->mode != FSP_ACTION_MODE_RC || !actionValid(a)) {
        return reject(s, FSP_ACTION_INVALID);
    }
    return FSP_ACTION_OK;
}

fspActionResult_e fspActionsEnqueue(fspActionScheduler_t *s, const fspPacketAction_t *a)
{
    // RC actions are applied on receipt and never scheduled
    if (a->mode == FSP_ACTION_MODE_RC || !actionValid(a)) {
        return reject(s, FSP_ACTION_INVALID);
    }
    const int32_t ahead = (int32_t)(a->applyTick - s->pidTick);
    if (ahead <= 0) {
        return reject(s, FSP_ACTION_LATE);
    }
    if ((uint32_t)ahead > s->maxAheadTicks) {
        return reject(s, FSP_ACTION_HORIZON);
    }
    if (s->count) {
        const fspPacketAction_t *last = &s->queue[(s->head + s->count - 1) % FSP_ACTION_QUEUE_CAPACITY];
        if ((int32_t)(a->applyTick - last->applyTick) <= 0) {
            return reject(s, FSP_ACTION_ORDER);
        }
    }
    if (s->count == FSP_ACTION_QUEUE_CAPACITY) {
        return reject(s, FSP_ACTION_FULL);
    }
    s->queue[(s->head + s->count) % FSP_ACTION_QUEUE_CAPACITY] = *a;
    s->count++;
    return FSP_ACTION_OK;
}

// Scheduled (rate and motor) actions need both authority levels
static bool authorityAllows(uint8_t authority)
{
    return (authority & FSP_AUTHORITY_RC) && (authority & FSP_AUTHORITY_CONTROL);
}

bool fspActionsOutputActive(const fspActionScheduler_t *s)
{
    return s->haveActive && authorityAllows(s->authority);
}

static void trackPeriod(fspActionScheduler_t *s, uint32_t nowUs)
{
    if (!s->haveTick) {
        s->haveTick = true;
        s->lastTickUs = nowUs;
        return;
    }
    const uint64_t dtNs = (uint64_t)(nowUs - s->lastTickUs) * 1000U;
    s->lastTickUs = nowUs;
    if (2 * dtNs > 3ULL * s->periodNs) {
        // Ticks the scheduler dropped between this and the previous tick
        s->stats.skippedTicks += (uint32_t)((dtNs + s->periodNs / 2) / s->periodNs - 1);
    } else {
        s->periodNs = (uint32_t)((int32_t)s->periodNs + (((int32_t)dtNs - (int32_t)s->periodNs) >> FSP_PERIOD_EMA_SHIFT));
    }
    s->stats.pidPeriodNs = s->periodNs;
}

void fspActionsTick(fspActionScheduler_t *s, uint32_t nowUs, uint8_t authority)
{
    s->pidTick++;
    trackPeriod(s, nowUs);

    // Any authority change discards queued and active actions; the host has to start over.
    if (authority != s->authority) {
        s->head = s->count = 0;
        s->haveActive = false;
        s->authority = authority;
    }

    s->activated = false;
    while (s->count) {
        const fspPacketAction_t *a = &s->queue[s->head];
        const int32_t due = (int32_t)(s->pidTick - a->applyTick);
        if (due < 0) {
            break;
        }
        if (due == 0) {
            s->active = *a;
            s->haveActive = true;
            s->activated = true;
            s->activeTimeUs = nowUs;
        } else {
            // Only possible after skipped ticks; never apply late
            reject(s, FSP_ACTION_LATE);
        }
        s->head = (s->head + 1) % FSP_ACTION_QUEUE_CAPACITY;
        s->count--;
    }
    s->timedOut = s->haveActive && (uint32_t)(nowUs - s->activeTimeUs) >= s->timeoutUs;
}

void fspActionsFrameState(const fspActionScheduler_t *s, fspSensorFrame_t *frame)
{
    const uint8_t mode = s->haveActive ? s->active.mode : 0;
    frame->actionFlags = (authorityAllows(s->authority) ? FSP_ACTION_FLAG_AUTHORITY : 0) |
                         (fspActionsOutputActive(s) ? FSP_ACTION_FLAG_OUTPUT_ACTIVE : 0) |
                         (s->timedOut ? FSP_ACTION_FLAG_TIMED_OUT : 0) |
                         ((mode << FSP_ACTION_FLAG_MODE_SHIFT) & FSP_ACTION_FLAG_MODE_MASK);
    if (s->haveActive) {
        const uint32_t age = s->pidTick - s->active.applyTick;
        frame->actionAge = age >= FSP_ACTION_AGE_SATURATED ? FSP_ACTION_AGE_SATURATED : (uint8_t)age;
    } else {
        frame->actionAge = FSP_ACTION_AGE_NONE;
    }
    frame->queueDepth = s->count;
    frame->reserved = 0;
}

void fspActionsStatus(const fspActionScheduler_t *s, fspPacketStatus_t *status)
{
    status->actionTick = s->haveActive ? s->active.applyTick : 0;
    status->actionMode = s->haveActive ? s->active.mode : 0;
    status->authority = s->authority;
}
