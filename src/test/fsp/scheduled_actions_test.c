#include "fsp/fsp_actions.h"
#include <assert.h>
#include <stdio.h>

static fspActionPacket_t action(uint32_t sequence, uint32_t source, uint32_t target, uint8_t mode)
{
    fspActionPacket_t a = {0};
    a.sequence = sequence; a.sourceTick = source; a.applyTick = target; a.mode = mode;
    a.values[0] = 1000; a.values[1] = 2000; a.values[2] = 3000; a.values[3] = 4000;
    return a;
}
int main(void)
{
    fspActionScheduler_t s;
    fspActionsInit(&s);
    fspActionsTick(&s, 500, 500, false);
    for (uint32_t i = 1; i <= 4; i++) {
        fspActionPacket_t a = action(i, 1, 2+i, FSP_ACTION_MOTORS);
        assert(fspActionsEnqueue(&s, &a) == FSP_ACTION_OK);
    }
    fspActionPacket_t a = action(5, 1, 7, FSP_ACTION_MOTORS);
    assert(fspActionsEnqueue(&s, &a) == FSP_ACTION_FULL);
    for (uint32_t tick = 2; tick <= 6; tick++) {
        fspActionsTick(&s, tick * 500, 500, false);
        assert(!fspActionsOutputActive(&s));
        if (tick >= 3) {
            assert(s.status.consumedSequence == tick - 2);
            assert(s.status.consumedTick == tick && s.status.requestedTick == tick);
            assert(s.status.consumedTimeUs == tick * 500);
        }
    }
    assert(s.count == 0);
    a = action(6, 1, 6, FSP_ACTION_RATES);
    assert(fspActionsEnqueue(&s, &a) == FSP_ACTION_LATE);
    a.applyTick = 7; a.sequence = 4;
    assert(fspActionsEnqueue(&s, &a) == FSP_ACTION_ORDER);
    a.sequence = 6; a.applyTick = 1000;
    assert(fspActionsEnqueue(&s, &a) == FSP_ACTION_HORIZON);
    a.applyTick = 8; a.values[3] = -1;
    assert(fspActionsEnqueue(&s, &a) == FSP_ACTION_INVALID);
    a.values[3] = 5000;
    assert(fspActionsEnqueue(&s, &a) == FSP_ACTION_OK);
    fspActionsTick(&s, 3500, 500, true);
    assert(!s.haveActive && s.count == 0); // transition clears disarmed rehearsal
    a = action(7, 7, 9, FSP_ACTION_RATES);
    assert(fspActionsEnqueue(&s, &a) == FSP_ACTION_OK);
    fspActionsTick(&s, 4000, 500, true);
    assert(!fspActionsOutputActive(&s));
    fspActionsTick(&s, 4500, 500, true);
    assert(fspActionsOutputActive(&s) && !s.status.timedOut);
    fspActionsTick(&s, 204499, 500, true);
    assert(!s.status.timedOut);
    fspActionsTick(&s, 204500, 500, true);
    assert(s.status.timedOut);
    fspActionsTick(&s, 205000, 500, false);
    assert(!fspActionsOutputActive(&s));
    // Wrap both FC tick/time counters and action sequence numbers.
    fspActionsInit(&s);
    s.status.pidTick = UINT32_MAX - 2;
    a = action(UINT32_MAX, s.status.pidTick, 0, FSP_ACTION_MOTORS);
    assert(fspActionsEnqueue(&s, &a) == FSP_ACTION_OK);
    for (int i = 0; i < 3; i++) fspActionsTick(&s, UINT32_MAX - 499 + i * 500U, 500, false);
    assert(s.status.consumedTick == 0 && s.status.consumedSequence == UINT32_MAX);
    a = action(0, 0, 2, FSP_ACTION_RATES);
    assert(fspActionsEnqueue(&s, &a) == FSP_ACTION_OK);
    fspActionsTick(&s, 1500, 500, false);
    fspActionsTick(&s, 2000, 500, false);
    assert(s.status.consumedSequence == 0 && s.status.mode == FSP_ACTION_RATES);
    // A skipped target is discarded instead of being applied late.
    a = action(1, 2, 4, FSP_ACTION_MOTORS);
    assert(fspActionsEnqueue(&s, &a) == FSP_ACTION_OK);
    s.status.pidTick = 4;
    fspActionsTick(&s, 2500, 500, false);
    assert(s.status.lateCount == 1 && s.status.consumedSequence == 0);
    puts("scheduled actions: FIFO/deadline/order/overflow/authority/timeout/counter wrap passed");
}
