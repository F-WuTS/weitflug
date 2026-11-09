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

#include "platform.h"

#ifdef USE_FSP

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "fsp.h"

PG_REGISTER_WITH_RESET_TEMPLATE(fspConfig_t, fspConfig, PG_FSP_CONFIG, 1);

PG_RESET_TEMPLATE(fspConfig_t, fspConfig,
    .telemetry_hz = FSP_PERIOD_HZ,
    .telemetry_timeout_ms = 500,
    .action_timeout_ms = 200,
    .action_max_ahead_ticks = 128,
    .status_hz = 50,
);

#endif // USE_FSP
