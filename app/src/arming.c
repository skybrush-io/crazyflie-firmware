/*
 * This file is part of the Skybrush compatibility layer for the Crazyflie firmware.
 *
 * Copyright 2020-2022 CollMot Robotics Ltd.
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

#include "FreeRTOS.h"   /* bool is defined there */

#include "autoconf.h"

#include "arming.h"
#include "param.h"
#include "supervisor.h"

static bool isInit = false;

static struct {
  paramVarId_t stabilizerStop;
} paramIds;

void armingInit() {
  if (isInit) {
    return;
  }

  /* Retrieve the IDs of the log variables and parameters that we will need */
  paramIds.stabilizerStop = paramGetVarId("stabilizer", "stop");

  if (!PARAM_VARID_IS_VALID(paramIds.stabilizerStop)) {
    return;
  }

  isInit = true;
}

bool armingTest(void) {
  return isInit;
}

bool armingShouldArmAutomaticallyBeforeTakeoff(void) {
  return true;
}

bool armAutomaticallyIfNeeded(void) {
  if (!armingShouldArmAutomaticallyBeforeTakeoff() || supervisorIsArmed()) {
    return true;
  }

  return supervisorRequestArming(true);
}

bool armingShouldDisarmAutomaticallyAfterLanding(void) {
#ifdef CONFIG_MOTORS_START_DISARMED
  return true;
#else
  return false;
#endif
}

void armingForceDisarm() {
  supervisorRequestArming(false);
  armingBlockMotors();
}

void armingBlockMotors() {
  paramSetInt(paramIds.stabilizerStop, 1);
}

void armingUnblockMotors() {
  paramSetInt(paramIds.stabilizerStop, 0);
}
