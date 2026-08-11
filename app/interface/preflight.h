/*
 * Crazyflie on-board preflight check module.
 *
 * This file is part of the Skybrush compatibility layer for the Crazyflie
 * firmware.
 *
 * Copyright 2019-2022 CollMot Robotics Ltd.
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

#ifndef __PREFLIGHT_H__
#define __PREFLIGHT_H__

#include <stdbool.h>
#include <stdint.h>

/**
 * Enum describing the various preflight tests.
 */
typedef enum {
  PREFLIGHT_CHECK_BATTERY = 0,
  PREFLIGHT_CHECK_SENSORS = 1,
  PREFLIGHT_CHECK_KALMAN_FILTER = 2,
  PREFLIGHT_CHECK_POSITIONING = 3,
  PREFLIGHT_CHECK_HOME = 4,
  PREFLIGHT_CHECK_TRAJECTORY_AND_LIGHTS = 5,
  PREFLIGHT_CHECK_SUPERVISOR = 6
} preflight_check_t;

/**
 * Enum describing the different results that a preflight check may return.
 * "Off" should be used for tests that are not relevant, "Fail" should be
 * returned for test failures and "Pass" should be returned for successful
 * preflight checks. If a preflight check takes a longer time, the test may
 * also return "Wait".
 */
typedef enum result_e {
  preflightResultOff = 0,
  preflightResultFail = 1,
  preflightResultWait = 2,
  preflightResultPass = 3
} preflight_check_result_t;

/**
 * Variable type that contains the result of all preflight checks. The results
 * are filled in from the LSB; each result is stored in two bits. In other
 * words, the result of the first test is stored in bits 0-1, the result of the
 * second test is stored in bits 2-3 and so on.
 */
typedef uint16_t preflight_check_status_t;

/**
 * Initializer function that should be called early during the startup process.
 */
void preflightInit(void);

/**
 * Tests whether the preflight module is ready to be used.
 */
bool preflightTest(void);

/**
 * Sets whether the preflight module is enabled.
 */
void preflightSetEnabled(bool value);

/**
 * Sets whether the preflight module is forced to pass.
 */
void preflightSetForcedToPass(bool value);

/**
 * Returns the detailed results of the preflight tests.
 */
preflight_check_status_t getPreflightCheckStatus();

/**
 * Returns the detailed result of a single preflight tests.
 *
 * Use the symbolic constants from the preflight_check_t enum when calling this
 * function.
 */
preflight_check_result_t getSinglePreflightCheckStatus(uint8_t index);

/**
 * Returns a summary of the preflight tests.
 *
 * If there are no preflight tests enabled, the result will be "Off".
 * Otherwise, when at least one test is failing, the result will be "Fail".
 * Otherwise, when at least one test is still in progress, the result will be
 * "Wait". In all other cases, the result will be "Pass".
 */
preflight_check_result_t getPreflightCheckSummary();

/**
 * Returns whether a single preflight check is failing.
 *
 * Use the symbolic constants from the preflight_check_t enum when calling this
 * function.
 */
bool isPreflightCheckFailing(uint8_t index);

/**
 * Returns whether a single preflight check is passing.
 *
 * Use the symbolic constants from the preflight_check_t enum when calling this
 * function.
 */
bool isPreflightCheckPassing(uint8_t index);

/**
 * Resets the internal Kalman filter to the home position of the drone.
 */
void preflightResetKalmanFilterToHome();

#endif // __PREFLIGHT_H__
