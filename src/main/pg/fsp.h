/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>

#include "pg/pg.h"

typedef struct fspConfig_s {
    uint16_t telemetry_hz;           // state packet rate and FSP task rate
    uint16_t telemetry_timeout_ms;   // telemetry stops this long after the last heartbeat or action
    uint16_t action_timeout_ms;      // outputs go neutral this long after the last applied action
    uint16_t action_max_ahead_ticks; // actions further ahead are rejected with FSP_ACTION_HORIZON
    uint16_t status_hz;              // status packet rate (attitude, aux, battery, scales, exact action tick)
} fspConfig_t;

PG_DECLARE(fspConfig_t, fspConfig);
