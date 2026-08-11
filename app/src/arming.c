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

#include "autoconf.h"

#include "arming.h"
#include "param.h"
#include "supervisor.h"

static bool isInit = false;

static struct {
  paramVarId_t motorPowerSetEnable;
  paramVarId_t motorPowerSetM1;
} paramIds;

void armingInit() {
  if (isInit) {
    return;
  }

  /* Retrieve the IDs of the parameters that we will need */
  paramIds.motorPowerSetEnable = paramGetVarId("motorPowerSet", "enable");
  paramIds.motorPowerSetM1 = paramGetVarId("motorPowerSet", "m1");

  if (
      !PARAM_VARID_IS_VALID(paramIds.motorPowerSetEnable) ||
      !PARAM_VARID_IS_VALID(paramIds.motorPowerSetM1)
  ) {
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
#ifdef CONFIG_MOTORS_REQUIRE_ARMING
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
  paramSetInt(paramIds.motorPowerSetM1, 0);
  paramSetInt(paramIds.motorPowerSetEnable, 2);
}

void armingUnblockMotors() {
  paramSetInt(paramIds.motorPowerSetEnable, 0);
}
