/*
 * Utility functions in the Crazyflie firmware
 *
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

#ifndef __SKYBRUSH_DRONE_SHOW_UTILS_H__
#define __SKYBRUSH_DRONE_SHOW_UTILS_H__

#include <stdbool.h>

/**
 * Returns whether at least one motor is running.
 */
bool motorsAreRunning(void);

#endif // __SKYBRUSH_ARMING_H__
