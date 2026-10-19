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

#pragma once

#ifdef USE_BANK_MODE

#include <stdint.h>

#include "pg/pg.h"

typedef struct bankModeConfig_s {
    uint16_t midSpeed;              // airspeed at the centre of the Speed stick, x10 m/s
    uint16_t cornerRate;            // deg/s, the track's turn rate full Bank slows the craft down for
} bankModeConfig_t;

PG_DECLARE(bankModeConfig_t, bankModeConfig);

#endif // USE_BANK_MODE
