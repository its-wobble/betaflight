/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "platform.h"

#ifdef USE_BANK_MODE

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "bank_mode.h"

PG_REGISTER_WITH_RESET_TEMPLATE(bankModeConfig_t, bankModeConfig, PG_BANK_MODE_CONFIG, 2);

PG_RESET_TEMPLATE(bankModeConfig_t, bankModeConfig,
    .thrustToWeight = 200,
    .maxBankAngle = 45,
    .maxTiltAngle = 30,
    .stickDeadband = 5,
    .headCheckRange = 90,
    .headCheckRate = 360,
    .ballKp = 300,
    .ballKi = 300,
    .ballDeadband = 20,
    .yawMaxRate = 360,
    .speedKp = 200,
    .speedDeadband = 20,
);

#endif // USE_BANK_MODE
