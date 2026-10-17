#pragma once
#include <stdbool.h>
#include <stdint.h>

// PT2 telemetry filter; 1.553773974 corrects the two-pole -3 dB cutoff.
typedef struct {
    float first[3], second[3], gain;
    unsigned rateHz;
    uint32_t timestampUs;
    bool initialized;
} fspImuPt2Filter_t;

static inline void fspImuPt2FilterUpdate(fspImuPt2Filter_t *s, const float values[3], unsigned rateHz, uint32_t nowUs)
{
    if (!rateHz) return;
    if (s->rateHz != rateHz) {
        const float omega = 6.28318530718f * 80.0f * 1.553773974f;
        s->gain = omega / (rateHz + omega);
        s->rateHz = rateHz;
    }
    for (unsigned axis = 0; axis < 3; ++axis) {
        if (!s->initialized) s->first[axis] = s->second[axis] = values[axis];
        s->first[axis] += s->gain * (values[axis] - s->first[axis]);
        s->second[axis] += s->gain * (s->first[axis] - s->second[axis]);
    }
    s->timestampUs = nowUs;
    s->initialized = true;
}

static inline bool fspImuPt2FilterFresh(const fspImuPt2Filter_t *s, uint32_t nowUs)
{
    return s->initialized && (uint32_t)(nowUs - s->timestampUs) <= 4000U;
}
