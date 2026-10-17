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
    uint16_t thrustToWeight;        // full-collective thrust / weight, x100; 1 / collective^1.5 in a hover
    uint16_t midSpeed;              // speed at the centre of the Speed stick, x100 g of drag
    uint8_t headCheckRange;         // degrees
    uint8_t lookTurn;               // percent, 0 = look dot, 100 = look turn
} bankModeConfig_t;

PG_DECLARE(bankModeConfig_t, bankModeConfig);

#endif // USE_BANK_MODE
