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
    uint16_t thrustToWeight;        // max thrust / weight, x100 (200 = 2.0)
    uint8_t maxBankAngle;           // bank angle at full roll stick, degrees
    uint8_t maxTiltAngle;           // forward/backward tilt at full pitch stick, degrees
    uint8_t stickDeadband;          // deadband around stick centre, percent of full deflection
    uint8_t headCheckRange;         // nose offset from the travel direction at full yaw stick, degrees
    uint16_t headCheckRate;         // head-check slew and return rate, degrees per second
    uint16_t ballKp;                // yaw rate per g of lateral specific force (the ball), degrees per second per g
    uint16_t ballKi;                // ball integral gain, degrees per second squared per g
    uint16_t ballDeadband;          // lateral specific force ignored around the centre of the ball, x1000 g
    uint16_t yawMaxRate;            // limit of the yaw rate command, degrees per second
    uint16_t speedKp;               // forward speed error gain, x100 (200 = 2.0): the braking strength at centre stick
    uint16_t speedDeadband;         // drag ignored around zero when estimating the forward speed, x1000 g
} bankModeConfig_t;

PG_DECLARE(bankModeConfig_t, bankModeConfig);

#endif // USE_BANK_MODE
