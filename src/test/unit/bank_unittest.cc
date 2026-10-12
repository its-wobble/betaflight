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

#include <stdint.h>

extern "C" {
    #include "platform.h"

    #include "fc/rc_modes.h"
    #include "fc/runtime_config.h"
    #include "flight/bank_mode.h"
    #include "io/beeper.h"
    #include "pg/bank_mode.h"
    #include "pg/pg.h"

    extern uint8_t __config_start;
    extern uint8_t __config_end;

    void beeperConfirmationBeeps(uint8_t) {}
}

#include "gtest/gtest.h"

TEST(BankModeTest, ConfigDefaults)
{
    pgResetAll();
    EXPECT_EQ(200, bankModeConfig()->thrustToWeight);
}

TEST(BankModeTest, ModeBitIsDistinct)
{
    const uint16_t others = ANGLE_MODE | HORIZON_MODE | MAG_MODE | ALT_HOLD_MODE | POS_HOLD_MODE
        | HEADFREE_MODE | CHIRP_MODE | PASSTHRU_MODE | FAILSAFE_MODE | GPS_RESCUE_MODE
        | AUTOPILOT_MODE | LAUNCH_MODE;
    EXPECT_EQ(0, BANK_MODE & others);
    EXPECT_EQ(BANK_MODE, 1 << 14);
}

TEST(BankModeTest, BoxIsMappedLikeAnAttitudeMode)
{
    const uint8_t boxIdToFlightModeMap[] = BOXID_TO_FLIGHT_MODE_MAP_INITIALIZER;

    EXPECT_LE(BOXBANK, BOXID_FLIGHTMODE_LAST);
    EXPECT_EQ(BANK_MODE, 1 << boxIdToFlightModeMap[BOXBANK]);
}

TEST(BankModeTest, SkeletonOnlyMirrorsModeFlag)
{
    flightModeFlags = 0;
    bankModeUpdate(0);
    EXPECT_FALSE(bankModeIsActive());

    ENABLE_FLIGHT_MODE(BANK_MODE);
    bankModeUpdate(1000);
    EXPECT_TRUE(bankModeIsActive());
    EXPECT_EQ(BANK_MODE, flightModeFlags);

    DISABLE_FLIGHT_MODE(BANK_MODE);
    bankModeUpdate(2000);
    EXPECT_FALSE(bankModeIsActive());
    EXPECT_EQ(0, flightModeFlags);
}
