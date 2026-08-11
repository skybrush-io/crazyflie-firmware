/*
 * This file is part of the Skybrush compatibility layer for the Crazyflie firmware.
 *
 * Copyright 2026 CollMot Robotics Ltd.
 *
 * This app is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This app is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include "autoconf.h"
#include "drone_show_utils.h"
#include "motors.h"

bool motorsAreRunning(void)
{
    for (uint8_t i = 0; i < NBR_OF_MOTORS; i++) {
        if (motorsGetRatio(i) > 512) {
            return true;
        }
    }

    return false;
}
