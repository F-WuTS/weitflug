#include <cstdint>
#include <climits>
extern "C" {
#include "platform.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "rx/rx.h"
#include "rx/msp.h"
#include "rx/msp_override.h"
#include "drivers/time.h"
}
#include "gtest/gtest.h"

static timeUs_t nowUs;
static bool overrideActive;
static float radioRead(const rxRuntimeState_t *, uint8_t channel)
{
    return 1200 + channel * 50;
}
extern "C" timeUs_t micros(void) { return nowUs; }
extern "C" bool IS_RC_MODE_ACTIVE(boxId_e id)
{
    return id == BOXMSPOVERRIDE && overrideActive;
}

TEST(MspOverride, StartupTimeoutRecoveryAndRadioSelection)
{
    rxRuntimeState_t runtime = {};
    runtime.rcReadRawFn = radioRead;
    rxConfig_t config = {};
    for (unsigned c = 0; c < RX_MAPPABLE_CHANNEL_COUNT; ++c) config.rcmap[c] = c;
    config.rx_min_usec = 885;
    config.rx_max_usec = 2115;
    config.msp_override_channels_mask = 0x1f; // Primary channels and AUX1.
    overrideActive = true;
    nowUs = 0;
    const uint16_t fallback[] = {1500, 1500, 1500, 1000};
    auto expectFallback = [&]() {
        for (uint8_t c = 0; c < PRIMARY_CHANNEL_COUNT; ++c)
            EXPECT_EQ(fallback[c], rxMspOverrideReadRawRc(&runtime, &config, c));
        EXPECT_EQ(radioRead(&runtime, 4), rxMspOverrideReadRawRc(&runtime, &config, 4));
    };
    expectFallback(); // No command has ever arrived, including at boot time zero.
    nowUs = 1000000;
    expectFallback();

    const uint16_t command[] = {1600, 1400, 1550, 1250, 1800};
    rxMspFrameReceive(command, 5);
    auto expectCommand = [&]() {
        for (uint8_t c = 0; c < 5; ++c)
            EXPECT_EQ(command[c], rxMspOverrideReadRawRc(&runtime, &config, c));
    };
    expectCommand();
    nowUs += 199999;
    expectCommand();
    ++nowUs;
    expectFallback(); // Exactly 200 ms is expired.
    nowUs += 500000;
    expectFallback();
    rxMspFrameReceive(command, 5);
    expectCommand(); // Recovery without releasing the switch.

    overrideActive = false;
    for (uint8_t c = 0; c < 5; ++c)
        EXPECT_EQ(radioRead(&runtime, c), rxMspOverrideReadRawRc(&runtime, &config, c));
    overrideActive = true;
    config.msp_override_channels_mask &= ~(1U << PITCH);
    EXPECT_EQ(radioRead(&runtime, PITCH), rxMspOverrideReadRawRc(&runtime, &config, PITCH));
    nowUs += 200000;
    EXPECT_EQ(radioRead(&runtime, PITCH), rxMspOverrideReadRawRc(&runtime, &config, PITCH));
    config.msp_override_channels_mask |= 1U << PITCH;

    nowUs = UINT32_MAX - 99999U;
    rxMspFrameReceive(command, 5);
    nowUs = 99999;
    expectCommand(); // 199999 us across timer wrap.
    nowUs = 100000;
    expectFallback();
}

TEST(MspOverride, FallbackFollowsReceiverChannelMap)
{
    rxRuntimeState_t runtime = {};
    runtime.rcReadRawFn = radioRead;
    rxConfig_t config = {};
    config.rx_min_usec = 885;
    config.rx_max_usec = 2115;
    config.msp_override_channels_mask = 0xff;
    overrideActive = true;
    nowUs = 3000000;
    const uint16_t command[] = {1600, 1400, 1550, 1250, 1800};
    rxMspFrameReceive(command, 5);
    nowUs += 200000;
    // Entries are raw indices for logical roll, pitch, yaw, throttle.
    const uint8_t maps[][4] = {{0, 1, 3, 2}, {1, 2, 3, 0}, {4, 5, 7, 6}};
    const uint16_t expected[][8] = {
        {1500, 1500, 1000, 1500, 1400, 1450, 1500, 1550}, // AETR1234
        {1000, 1500, 1500, 1500, 1400, 1450, 1500, 1550}, // TAER1234
        {1200, 1250, 1300, 1350, 1500, 1500, 1000, 1500}  // 1234AETR
    };
    for (unsigned m = 0; m < 3; ++m) {
        for (unsigned c = 0; c < 4; ++c) config.rcmap[c] = maps[m][c];
        for (uint8_t c = 0; c < 8; ++c)
            EXPECT_EQ(expected[m][c], rxMspOverrideReadRawRc(&runtime, &config, c));
    }
}
