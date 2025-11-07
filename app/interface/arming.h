/*
 * Arming-related functions in the Crazyflie firmware
 *
 * This file is part of the Skybrush compatibility layer for the Crazyflie firmware.
 *
 * Copyright 2021-2022 CollMot Robotics Ltd.
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

#ifndef __SKYBRUSH_ARMING_H__
#define __SKYBRUSH_ARMING_H__

/**
 * Initializes the arming module.
 */
void armingInit(void);

/**
 * Tests whether the arming module is ready to be used.
 */
bool armingTest(void);

/**
 * Arms the drone if it is set to automatic arming before takeoff and it is
 * not armed yet.
 *
 * Returns whether the arming was successful or if no action should have
 * been taken.
 */
bool armAutomaticallyIfNeeded(void);

/**
 * Force-disarm the drone even if it is currently force-armed.
 */
void armingForceDisarm(void);

/**
 * Returns whether the drone arms itself automatically before takeoff.
 */
bool armingShouldArmAutomaticallyBeforeTakeoff(void);

/**
 * Returns whether the drone disarms itself automatically after landing.
 */
bool armingShouldDisarmAutomaticallyAfterLanding(void);

/**
 * Blocks the motors immediately (i.e. prevents them from spinning).
 */
void armingBlockMotors();

/**
 * Unblocks the motors if they are currently blocked.
 */
void armingUnblockMotors();

#endif // __SKYBRUSH_ARMING_H__
