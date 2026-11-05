#include "fsp/fsp_actions.h"
#include <assert.h>
#include <stdio.h>

#define PERIOD_US 500U
#define TIMEOUT_US 200000U
#define MAX_AHEAD 128U
#define ALL (FSP_AUTHORITY_RC | FSP_AUTHORITY_CONTROL)

static fspPacketAction_t action(uint32_t applyTick, uint8_t mode)
{
    fspPacketAction_t a = {0};
    a.applyTick = applyTick;
    a.mode = mode;
    for (unsigned i = 0; i < 4; i++) {
        a.values[i] = mode == FSP_ACTION_MODE_RC ? 1500 : 1000 * (i + 1);
    }
    return a;
}

// Tick at the nominal period: tick n happens at n * PERIOD_US
static void tickAt(fspActionScheduler_t *s, uint32_t tick, uint8_t authority)
{
    fspActionsTick(s, tick * PERIOD_US, authority);
    assert(s->pidTick == tick);
}

static void testQueueAndValidation(void)
{
    fspActionScheduler_t s;
    fspActionsInit(&s, TIMEOUT_US, MAX_AHEAD, PERIOD_US);
    tickAt(&s, 1, 0);
    for (uint32_t i = 1; i <= 4; i++) {
        fspPacketAction_t a = action(2 + i, FSP_ACTION_MODE_MOTORS);
        assert(fspActionsEnqueue(&s, &a) == FSP_ACTION_OK);
    }
    fspPacketAction_t a = action(7, FSP_ACTION_MODE_MOTORS);
    assert(fspActionsEnqueue(&s, &a) == FSP_ACTION_FULL);

    // Applied exactly on their tick, in order; no authority so no outputs
    for (uint32_t tick = 2; tick <= 6; tick++) {
        tickAt(&s, tick, 0);
        assert(!fspActionsOutputActive(&s));
        assert(s.activated == (tick >= 3));
        if (tick >= 3) assert(s.active.applyTick == tick && s.activeTimeUs == tick * PERIOD_US);
    }
    assert(s.count == 0);

    a = action(6, FSP_ACTION_MODE_RATES);
    assert(fspActionsEnqueue(&s, &a) == FSP_ACTION_LATE);
    a.applyTick = 6 + MAX_AHEAD + 1;
    assert(fspActionsEnqueue(&s, &a) == FSP_ACTION_HORIZON);
    a.applyTick = 6 + MAX_AHEAD;
    assert(fspActionsEnqueue(&s, &a) == FSP_ACTION_OK);
    a.applyTick = 8;
    assert(fspActionsEnqueue(&s, &a) == FSP_ACTION_ORDER);

    // Value ranges and mode/reserved validation
    fspActionsInit(&s, TIMEOUT_US, MAX_AHEAD, PERIOD_US);
    a = action(5, FSP_ACTION_MODE_RATES); a.values[3] = -1;
    assert(fspActionsEnqueue(&s, &a) == FSP_ACTION_INVALID);
    a = action(5, FSP_ACTION_MODE_RATES); a.values[0] = -20000;
    assert(fspActionsEnqueue(&s, &a) == FSP_ACTION_OK);
    a = action(6, FSP_ACTION_MODE_MOTORS); a.values[1] = 10001;
    assert(fspActionsEnqueue(&s, &a) == FSP_ACTION_INVALID);
    a = action(6, 0);
    assert(fspActionsEnqueue(&s, &a) == FSP_ACTION_INVALID);
    a = action(6, FSP_ACTION_MODE_MOTORS); a.reserved[2] = 1;
    assert(fspActionsEnqueue(&s, &a) == FSP_ACTION_INVALID);
    a = action(6, FSP_ACTION_MODE_RC); // RC actions are never scheduled
    assert(fspActionsEnqueue(&s, &a) == FSP_ACTION_INVALID);

    // Counters are disjoint
    fspActionsInit(&s, TIMEOUT_US, MAX_AHEAD, PERIOD_US);
    a = action(0, FSP_ACTION_MODE_RATES);
    assert(fspActionsEnqueue(&s, &a) == FSP_ACTION_LATE);
    a = action(1, 0);
    assert(fspActionsEnqueue(&s, &a) == FSP_ACTION_INVALID);
    assert(s.stats.lateCount == 1 && s.stats.rejectedCount == 1 && s.stats.fullCount == 0);
}

static void testAuthorityAndTimeout(void)
{
    fspActionScheduler_t s;
    fspActionsInit(&s, TIMEOUT_US, MAX_AHEAD, PERIOD_US);
    tickAt(&s, 1, 0);
    fspPacketAction_t a = action(3, FSP_ACTION_MODE_RATES);
    assert(fspActionsEnqueue(&s, &a) == FSP_ACTION_OK);
    tickAt(&s, 2, ALL);
    assert(s.count == 0); // authority change flushes the queue

    a = action(4, FSP_ACTION_MODE_RATES);
    assert(fspActionsEnqueue(&s, &a) == FSP_ACTION_OK);
    tickAt(&s, 3, ALL);
    assert(!fspActionsOutputActive(&s));
    tickAt(&s, 4, ALL);
    assert(fspActionsOutputActive(&s) && !s.timedOut);
    fspActionsTick(&s, 4 * PERIOD_US + TIMEOUT_US - 1, ALL);
    assert(!s.timedOut);
    fspActionsTick(&s, 4 * PERIOD_US + TIMEOUT_US, ALL);
    assert(s.timedOut);

    // Losing control authority drops rates; RC authority alone does not drive them
    fspActionsTick(&s, 4 * PERIOD_US + TIMEOUT_US + PERIOD_US, FSP_AUTHORITY_RC);
    assert(!s.haveActive && !fspActionsOutputActive(&s));
    a = action(s.pidTick + 1, FSP_ACTION_MODE_RATES);
    assert(fspActionsEnqueue(&s, &a) == FSP_ACTION_OK);
    fspActionsTick(&s, s.lastTickUs + PERIOD_US, FSP_AUTHORITY_RC);
    assert(s.haveActive && !fspActionsOutputActive(&s));
    fspSensorFrame_t frame = {0};
    fspActionsFrameState(&s, &frame);
    assert(frame.actionFlags == 0);

    // Sensor frame state mirrors the active action
    a = action(s.pidTick + 2, FSP_ACTION_MODE_MOTORS);
    assert(fspActionsEnqueue(&s, &a) == FSP_ACTION_OK);
    fspActionsTick(&s, s.lastTickUs + PERIOD_US, ALL); // authority change flushes
    a = action(s.pidTick + 1, FSP_ACTION_MODE_MOTORS);
    assert(fspActionsEnqueue(&s, &a) == FSP_ACTION_OK);
    fspActionsTick(&s, s.lastTickUs + PERIOD_US, ALL);
    assert(fspActionsOutputActive(&s));
    fspActionsFrameState(&s, &frame);
    assert(frame.actionTick == s.pidTick && frame.actionMode == FSP_ACTION_MODE_MOTORS);
    assert(frame.actionFlags == (FSP_ACTION_FLAG_AUTHORITY | FSP_ACTION_FLAG_OUTPUT_ACTIVE));
    assert(frame.queueDepth == 0);
}

static void testRcImmediate(void)
{
    fspActionScheduler_t s;
    fspActionsInit(&s, TIMEOUT_US, MAX_AHEAD, PERIOD_US);
    for (uint32_t tick = 1; tick <= 10; tick++) tickAt(&s, tick, ALL);
    fspPacketAction_t queued = action(12, FSP_ACTION_MODE_RATES);
    assert(fspActionsEnqueue(&s, &queued) == FSP_ACTION_OK);

    // Accepted whatever the apply tick, without touching the scheduler
    const uint32_t ticks[] = {0, 5, 10, 11, 10 + MAX_AHEAD + 1, UINT32_MAX};
    for (unsigned i = 0; i < sizeof(ticks) / sizeof(ticks[0]); i++) {
        fspPacketAction_t a = action(ticks[i], FSP_ACTION_MODE_RC);
        assert(fspActionsCheckRc(&s, &a) == FSP_ACTION_OK);
    }
    assert(s.count == 1 && !s.haveActive);
    assert(s.stats.lateCount == 0 && s.stats.rejectedCount == 0);

    // Values and reserved bytes are still validated
    fspPacketAction_t a = action(11, FSP_ACTION_MODE_RC); a.values[2] = 999;
    assert(fspActionsCheckRc(&s, &a) == FSP_ACTION_INVALID);
    a = action(11, FSP_ACTION_MODE_RC); a.values[3] = 2001;
    assert(fspActionsCheckRc(&s, &a) == FSP_ACTION_INVALID);
    a = action(11, FSP_ACTION_MODE_RC); a.reserved[0] = 1;
    assert(fspActionsCheckRc(&s, &a) == FSP_ACTION_INVALID);
    a = action(11, FSP_ACTION_MODE_RATES);
    assert(fspActionsCheckRc(&s, &a) == FSP_ACTION_INVALID);
    assert(s.stats.rejectedCount == 4 && s.count == 1);
}

static void testWrapAndLate(void)
{
    fspActionScheduler_t s;
    fspActionsInit(&s, TIMEOUT_US, MAX_AHEAD, PERIOD_US);
    s.pidTick = UINT32_MAX - 2;
    fspPacketAction_t a = action(0, FSP_ACTION_MODE_MOTORS);
    assert(fspActionsEnqueue(&s, &a) == FSP_ACTION_OK);
    for (int i = 0; i < 3; i++) fspActionsTick(&s, UINT32_MAX - 999 + i * PERIOD_US, 0);
    assert(s.pidTick == 0 && s.activated && s.active.applyTick == 0);

    // A target skipped by the scheduler is discarded, never applied late
    a = action(3, FSP_ACTION_MODE_MOTORS);
    assert(fspActionsEnqueue(&s, &a) == FSP_ACTION_OK);
    s.pidTick = 3;
    fspActionsTick(&s, s.lastTickUs + PERIOD_US, 0);
    assert(s.stats.lateCount == 1 && s.active.applyTick == 0);
}

static void testPeriodAndSkips(void)
{
    fspActionScheduler_t s;
    fspActionsInit(&s, TIMEOUT_US, MAX_AHEAD, PERIOD_US);
    // Real loop runs 1% slow: 505 us; EMA converges towards 505000 ns
    uint32_t now = 0;
    for (int i = 0; i < 1000; i++) fspActionsTick(&s, now += 505, 0);
    assert(s.stats.pidPeriodNs > 504000 && s.stats.pidPeriodNs <= 505000);
    assert(s.stats.skippedTicks == 0);
    // Gap of 3 periods = 2 ticks lost; the period estimate is not disturbed
    const uint32_t period = s.stats.pidPeriodNs;
    fspActionsTick(&s, now += 3 * 505, 0);
    assert(s.stats.skippedTicks == 2 && s.stats.pidPeriodNs == period);
    // Survives micros() wrap
    s.lastTickUs = UINT32_MAX - 100;
    fspActionsTick(&s, UINT32_MAX - 100 + 505, 0);
    assert(s.stats.skippedTicks == 2);
}

int main(void)
{
    testQueueAndValidation();
    testAuthorityAndTimeout();
    testRcImmediate();
    testWrapAndLate();
    testPeriodAndSkips();
    puts("scheduled actions: queue/validation/authority/timeout/rc/wrap/late/period/skips passed");
}
