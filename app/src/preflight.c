/*
 * Crazyflie preflight check module
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

#include <float.h>
#include <math.h>

#include "autoconf.h"

#include "FreeRTOS.h"
#include "timers.h"

#include "crtp_commander_high_level.h"
#include "drone_show_utils.h"
#include "estimator_kalman.h"
#include "light_program.h"
#include "locodeck.h"
#include "log.h"
#include "motors.h"
#include "param.h"
#include "pm.h"
#include "preflight.h"
#include "pulse_processor.h"
#include "sensors.h"
#include "supervisor.h"
#include "worker.h"

#define DEBUG_MODULE "PREFLT"
#include "debug.h"

#ifdef CONFIG_PREFLIGHT_ENABLE_BATTERY_VOLTAGE_CHECK
#  define PREFLIGHT_MIN_BATTERY_VOLTAGE (CONFIG_PREFLIGHT_MIN_BATTERY_VOLTAGE / 10.0f)
#else
#  define PREFLIGHT_MIN_BATTERY_VOLTAGE 0.0f
#endif

#ifdef CONFIG_PREFLIGHT_ENABLE_POSITIONING_CHECK
#  ifdef CONFIG_SHOW_POSITIONING_UWB
#    define PREFLIGHT_MIN_LOCO_ANCHOR_COUNT CONFIG_PREFLIGHT_MIN_LOCO_ANCHOR_COUNT
#    define PREFLIGHT_MIN_ACTIVE_LOCO_ANCHOR_COUNT CONFIG_PREFLIGHT_MIN_ACTIVE_LOCO_ANCHOR_COUNT
#  endif
#  ifdef CONFIG_SHOW_POSITIONING_LIGHTHOUSE
#    define PREFLIGHT_MIN_LH_BS_COUNT CONFIG_PREFLIGHT_MIN_LH_BS_COUNT
#    define PREFLIGHT_MIN_ACTIVE_LH_BS_COUNT CONFIG_PREFLIGHT_MIN_ACTIVE_LH_BS_COUNT
#  endif
#  ifdef CONFIG_SHOW_POSITIONING_ACTIVE_MARKER
#    define PREFLIGHT_CHECK_ACTIVE_MARKER_DECK 1
#  endif
#endif

#ifdef CONFIG_PREFLIGHT_ENABLE_TRAJECTORY_CHECK
#  define PREFLIGHT_MIN_TRAJECTORIES CONFIG_PREFLIGHT_MIN_TRAJECTORIES
#  define PREFLIGHT_MIN_LIGHT_PROGRAMS CONFIG_PREFLIGHT_MIN_LIGHT_PROGRAMS
#endif

#ifdef CONFIG_PREFLIGHT_ENABLE_HOME_POSITION_CHECK
#  define PREFLIGHT_MAX_HOME_POSITION_DISTANCE_XY_CM CONFIG_PREFLIGHT_MAX_HOME_POSITION_DISTANCE_XY_CM
#  define PREFLIGHT_MAX_HOME_POSITION_DISTANCE_Z_CM CONFIG_PREFLIGHT_MAX_HOME_POSITION_DISTANCE_Z_CM
#endif

#ifndef PREFLIGHT_CHECK_ACTIVE_MARKER_DECK
#  define PREFLIGHT_CHECK_ACTIVE_MARKER_DECK 0
#endif

#ifndef PREFLIGHT_MIN_BATTERY_VOLTAGE
#  define PREFLIGHT_MIN_BATTERY_VOLTAGE 0.0f
#endif

#ifndef PREFLIGHT_MIN_LH_BS_COUNT
#  define PREFLIGHT_MIN_LH_BS_COUNT 0
#endif

#ifndef PREFLIGHT_MIN_ACTIVE_LH_BS_COUNT
#  define PREFLIGHT_MIN_ACTIVE_LH_BS_COUNT 0
#endif

#ifndef PREFLIGHT_MIN_LOCO_ANCHOR_COUNT
#  define PREFLIGHT_MIN_LOCO_ANCHOR_COUNT 0
#endif

#ifndef PREFLIGHT_MIN_ACTIVE_LOCO_ANCHOR_COUNT
#  define PREFLIGHT_MIN_ACTIVE_LOCO_ANCHOR_COUNT 0
#endif

#ifndef PREFLIGHT_MIN_TRAJECTORIES
#  define PREFLIGHT_MIN_TRAJECTORIES 0
#endif

#ifndef PREFLIGHT_MIN_LIGHT_PROGRAMS
#  define PREFLIGHT_MIN_LIGHT_PROGRAMS 0
#endif

#ifndef PREFLIGHT_MAX_HOME_POSITION_DISTANCE_XY_CM
#  define PREFLIGHT_MAX_HOME_POSITION_DISTANCE_XY_CM 50
#endif

#ifndef PREFLIGHT_MAX_HOME_POSITION_DISTANCE_Z_CM
#  define PREFLIGHT_MAX_HOME_POSITION_DISTANCE_Z_CM 50
#endif

#define PREFLIGHT_CHECK_INTERVAL_MSEC 500
#define KALMAN_VARIANCE_LOG_LENGTH 10     /* 5 seconds @ 2 Hz */

/* Bitcraze uses a variance threshold of 0.001f in their example scripts,
 * but it is apparently too low for Lighthouse because the interference
 * between LH base stations occasionally creates a "spike" in the variance
 * plot. The spike typically has a duration < 1s and a height that sometimes
 * reaches 0.001f, making the Kalman preflight test fail temporarily */
#ifdef CONFIG_SHOW_POSITIONING_LIGHTHOUSE
#  define KALMAN_VARIANCE_THRESHOLD 0.002f
#elif defined(CONFIG_SHOW_TDOA_LARGE_AREA)
#  define KALMAN_VARIANCE_THRESHOLD 0.01f
#else
#  define KALMAN_VARIANCE_THRESHOLD 0.001f
#endif

static StaticTimer_t timerBuffer;

static bool isInit = false;

static bool isEnabled = false;
static bool isForcedToPass = false;
static bool kalmanFilterResetRequested = false;
static float homeCoordinate[3] = { 0, 0, -10000 };

static struct {
  logVarId_t kalmanPosVariance[3];
  logVarId_t yaw;
  logVarId_t lighthouseBsHasCalibrationData;
  logVarId_t lighthouseBsHasGeometryData;
  logVarId_t lighthouseBsActive;
} logIds;

static struct {
  paramVarId_t kalmanInitialX;
  paramVarId_t kalmanInitialY;
  paramVarId_t kalmanInitialZ;
  paramVarId_t kalmanResetEstimation;
  paramVarId_t activeMarkerDeckConnected;
} paramIds;

static preflight_check_status_t preflightCheckStatus = 0;
static preflight_check_result_t preflightCheckSummary = preflightResultOff;

static void preflightTimer(xTimerHandle timer);
static void preflightWorker(void* data);

static preflight_check_result_t calculatePreflightCheckSummary(uint16_t status);
static void requestKalmanFilterReset(void);

void preflightInit() {
  if (isInit) {
    return;
  }

  xTimerHandle timer = xTimerCreateStatic("preflightTimer", M2T(PREFLIGHT_CHECK_INTERVAL_MSEC), pdTRUE, NULL, preflightTimer, &timerBuffer);
  xTimerStart(timer, PREFLIGHT_CHECK_INTERVAL_MSEC);

  paramIds.kalmanInitialX = paramGetVarId("kalman", "initialX");
  paramIds.kalmanInitialY = paramGetVarId("kalman", "initialY");
  paramIds.kalmanInitialZ = paramGetVarId("kalman", "initialZ");
  paramIds.kalmanResetEstimation = paramGetVarId("kalman", "resetEstimation");
  paramIds.activeMarkerDeckConnected = paramGetVarId("deck", "bcActiveMarker");

  if (
    !PARAM_VARID_IS_VALID(paramIds.kalmanInitialX) ||
    !PARAM_VARID_IS_VALID(paramIds.kalmanInitialY) ||
    !PARAM_VARID_IS_VALID(paramIds.kalmanInitialZ) ||
    !PARAM_VARID_IS_VALID(paramIds.kalmanResetEstimation) ||
    !PARAM_VARID_IS_VALID(paramIds.activeMarkerDeckConnected)
  ) {
    /* required parameters missing from firmware */
    return;
  }

  logIds.kalmanPosVariance[0] = logGetVarId("kalman", "varPX");
  logIds.kalmanPosVariance[1] = logGetVarId("kalman", "varPY");
  logIds.kalmanPosVariance[2] = logGetVarId("kalman", "varPZ");
  logIds.yaw = logGetVarId("stateEstimate", "yaw");
  logIds.lighthouseBsActive = logGetVarId("lighthouse", "bsActive");
  logIds.lighthouseBsHasCalibrationData = logGetVarId("lighthouse", "bsCalVal");
  logIds.lighthouseBsHasGeometryData = logGetVarId("lighthouse", "bsGeoVal");

  if (
    !logVarIdIsValid(logIds.kalmanPosVariance[0]) ||
    !logVarIdIsValid(logIds.kalmanPosVariance[1]) ||
    !logVarIdIsValid(logIds.kalmanPosVariance[2]) ||
    !logVarIdIsValid(logIds.yaw) ||
    !logVarIdIsValid(logIds.lighthouseBsActive) ||
    !logVarIdIsValid(logIds.lighthouseBsHasCalibrationData) ||
    !logVarIdIsValid(logIds.lighthouseBsHasGeometryData)
  ) {
    /* required log variables missing from firmware */
    return;
  }

  isInit = true;
}

void preflightSetEnabled(bool value) {
  if (isEnabled == value) {
    return;
  }

  isEnabled = value;

  if (isEnabled) {
    /* avoid returning success for even a single iteration if there is a status
     * packet request between the time we enable the preflight checks and the
     * time it gets the first chance to run */
    preflightCheckSummary = preflightResultWait;
    preflightCheckStatus = preflightResultWait;
  } else {
    preflightCheckSummary = preflightResultOff;
    preflightCheckStatus = preflightResultOff;
  }
}

void preflightSetForcedToPass(bool value) {
  isForcedToPass = value;
}

bool preflightTest() {
  return isInit;
}

/**
 * Resets the Kalman filter to the home position of the drone.
 */
void preflightResetKalmanFilterToHome() {
  /* Let's be very careful here -- we should not reset the Kalman filter if
   * any of the motors are running because we might be in the air. */
  if (motorsAreRunning()) {
    DEBUG_PRINT("NOT resetting EKF: motors are running\n");
  } else if (homeCoordinate[2] <= -10000) {
    /* Nothing to do, this is okay; we haven't received our "real" home
     * coordinates yet. */
  } else {
    paramSetFloat(paramIds.kalmanInitialX, homeCoordinate[0]);
    paramSetFloat(paramIds.kalmanInitialY, homeCoordinate[1]);
    paramSetFloat(paramIds.kalmanInitialZ, homeCoordinate[2]);
    paramSetInt(paramIds.kalmanResetEstimation, 1);
  }
}

preflight_check_status_t getPreflightCheckStatus() {
  return preflightCheckStatus;
}

preflight_check_result_t getPreflightCheckSummary() {
  return preflightCheckSummary;
}

preflight_check_result_t getSinglePreflightCheckStatus(uint8_t index) {
  return preflightCheckStatus & (0b11 << index);
}

bool isPreflightCheckFailing(uint8_t index) {
  return getSinglePreflightCheckStatus(index) == preflightResultFail;
}

bool isPreflightCheckPassing(uint8_t index) {
  return getSinglePreflightCheckStatus(index) == preflightResultPass;
}

static preflight_check_result_t calculatePreflightCheckSummary(uint16_t x) {
  const uint16_t MASK = 0x5555;

  if (x == 0) {
    /* No preflight tests are enabled */
    return preflightResultOff;
  }

  /* Failure is marked by a "01" bit pattern in preflightCheckStatus.
   * We use bitwise operations to check for the pattern in all possible
   * locations in preflightCheckStatus as follows.
   *
   * First we let x = preflightCheckStatus for sake of simplicity. Then we
   * calculate x & (~x >> 1). This will ensure that bits 0, 2, 4, ... and so on
   * (enumerated from the right) become 1 if the corresponding preflight check
   * is failing. Bits 1, 3, 5 and so on are irrelevant so we mask them out.
   * Then we compare the masked result with zero; if the masked result is not
   * zero, there was at least one failing preflight check.
   */
  if (x & ((~x) >> 1) & MASK) {
    return preflightResultFail;
  }

  /* The same trick can be used to detect tests that are still in progress;
   * the corresponding marker is "10" so we need the same trick but we need to
   * calculate (~x) & (x >> 1) instead. */
  if ((~x) & (x >> 1) & MASK) {
    return preflightResultWait;
  }

  return preflightResultPass;
}

static void requestKalmanFilterReset(void) {
  kalmanFilterResetRequested = 1;
}

#define FAIL { return preflightResultFail; }
#define PASS { return preflightResultPass; }
#define SKIP { return preflightResultOff; }
#define FAIL_IF(condition) if (condition) { return preflightResultFail; }
#define FAIL_UNLESS(condition) if (!(condition)) { return preflightResultFail; }
#define PASS_IF(condition) if (condition) { return preflightResultPass; }
#define PASS_UNLESS(condition) if (!(condition)) { return preflightResultPass; }
#define PASS_IF_AND_ONLY_IF(condition) {      \
  return (condition) ? preflightResultPass : preflightResultFail; \
}

/**
 * Preflight check that determines whether the battery voltage is high enough
 * for a safe flight.
 */
static preflight_check_result_t testBattery() {
  PASS_IF_AND_ONLY_IF(pmGetBatteryVoltage() >= PREFLIGHT_MIN_BATTERY_VOLTAGE);
}

/**
 * Preflight check that tests whether the drone is close enough to its
 * designated home position (if the preflight system knows about a home
 * position).
 */
static preflight_check_result_t testHomePosition() {
  point_t pos;
  float dxy, dz, yaw;

  /* check yaw -- it must be close to zero */
  yaw = logGetFloat(logIds.yaw);
  if (yaw >= 180) {
    yaw -= 360.0f;
  }
  if (fabsf(yaw) > 10) {
    /* yaw diverged from zero */
    return preflightResultFail;
  }

  if (homeCoordinate[2] <= -10000) {
    /* No home coordinate set */
    return preflightResultOff;
  }

  estimatorKalmanGetEstimatedPos(&pos);

  dxy = hypot(pos.x - homeCoordinate[0], pos.y - homeCoordinate[1]);
  dz = fabsf(pos.z - homeCoordinate[2]);

  if (dxy > 5 || dz > 3) {
    /* Difference can't be _that_ large even with very noisy positioning data
     * so reset the Kalman filter to home */
    requestKalmanFilterReset();
  }

  PASS_IF_AND_ONLY_IF(
      dxy <= PREFLIGHT_MAX_HOME_POSITION_DISTANCE_XY_CM / 100.0f &&
      dz <= PREFLIGHT_MAX_HOME_POSITION_DISTANCE_Z_CM / 100.0f
  );
}

/**
 * Preflight check that tests whether the position estimate of the Kalman
 * filter is stable enough.
 */
static preflight_check_result_t testKalmanFilter() {
  static float varianceLog[KALMAN_VARIANCE_LOG_LENGTH][3];
  static uint16_t writeIndex;
  static uint8_t failureCounter = 0;
  static bool initialized = 0;
  static bool convergedAtLeastOnce = 0;

  uint16_t i, dim, count;
  float minValue, maxValue;
  bool diffTooLarge = false;
  preflight_check_result_t result = preflightResultWait;

  /* Perform a reset of the Kalman filter if needed */
  if (kalmanFilterResetRequested) {
    preflightResetKalmanFilterToHome();

    kalmanFilterResetRequested = 0;
    failureCounter = 0;
    convergedAtLeastOnce = 0;
    initialized = 0;  /* to clear the variance log */
  }

  /* Perform initializations if needed (at first invocation and after every
   * reset of the Kalman filter) */
  if (!initialized) {
    /* Clear Kalman filter variance log */
    for (dim = 0; dim < 3; dim++) {
      for (i = 0; i < KALMAN_VARIANCE_LOG_LENGTH; i++) {
        varianceLog[i][dim] = FLT_MAX;
      }
    }

    writeIndex = 0;
    initialized = 1;
  }

  /* Store the current variances from the filter */
  for (dim = 0; dim < 3; dim++) {
    varianceLog[writeIndex][dim] = logGetFloat(logIds.kalmanPosVariance[dim]);
  }

  /* Be optimistic :) */
  result = preflightResultPass;

  for (dim = 0; dim < 3; dim++) {
    /* Re-scan minimum and maximum */
    minValue = FLT_MAX;
    maxValue = FLT_MIN;
    count = 0;
    for (i = 0; i < KALMAN_VARIANCE_LOG_LENGTH; i++) {
      if (varianceLog[i][dim] != FLT_MAX) {
        count++;
        minValue = fminf(minValue, varianceLog[i][dim]);
        maxValue = fmaxf(maxValue, varianceLog[i][dim]);
      }
    }

    if (count < KALMAN_VARIANCE_LOG_LENGTH / 2) {
      /* Not enough samples yet. There's no point in checking further, break
       * out of the loop and return "wait" */
      result = preflightResultWait;
      break;
    }

    /* If we have passed the test at least once, and we are using Lighthouse,
     * and the maxValue is not too large, exit here because there are occasional
     * spikes in the variance with Lighthouse and we don't want that to influence
     * the result */
    if (PREFLIGHT_MIN_LH_BS_COUNT > 0 && convergedAtLeastOnce && maxValue < 10 * KALMAN_VARIANCE_THRESHOLD) {
      continue;
    }

    if ((maxValue - minValue) > KALMAN_VARIANCE_THRESHOLD) {
      /* Difference too large. If the trend is increasing, remember that so we
       * can trigger a reset later if needed */
      if (writeIndex < KALMAN_VARIANCE_LOG_LENGTH - 1) {
        if (varianceLog[writeIndex][dim] > varianceLog[writeIndex+1][dim]) {
          diffTooLarge = true;
          break;
        }
      } else {
        if (varianceLog[KALMAN_VARIANCE_LOG_LENGTH - 1][dim] > varianceLog[0][dim]) {
          diffTooLarge = true;
          break;
        }
      }
    }
  }

  /* Advance the write pointer */
  writeIndex++;
  if (writeIndex >= KALMAN_VARIANCE_LOG_LENGTH) {
    writeIndex = 0;
  }

  /* If the variance difference between min and max is too large and it has been
   * so for a long while now, reset the filter */
  if (diffTooLarge) {
    failureCounter++;
    if (failureCounter > 5000 / PREFLIGHT_CHECK_INTERVAL_MSEC) {
      requestKalmanFilterReset();
      failureCounter = 0;
    }
    result = preflightResultWait;
  } else {
    failureCounter = 0;
  }

  /* If we have passed the test, remember that we have passed at least once
   * so we can take this into account when using Lighthouse */
  if (result == preflightResultPass) {
    convergedAtLeastOnce = true;
  }

  return result;
}

/**
 * Preflight check that validates the configuration of the positioning system.
 */
static preflight_check_result_t testPositioningSystem() {
  uint8_t anchorList[PREFLIGHT_MIN_LOCO_ANCHOR_COUNT];
  uint8_t numActive = 0, numEnabled = 0;

  if (PREFLIGHT_MIN_LOCO_ANCHOR_COUNT > 0) {
    /* Positioning system is UWB */
    numEnabled = locoDeckGetAnchorIdList(anchorList, PREFLIGHT_MIN_LOCO_ANCHOR_COUNT);
    FAIL_IF(numEnabled < PREFLIGHT_MIN_LOCO_ANCHOR_COUNT);

    if (PREFLIGHT_MIN_ACTIVE_LOCO_ANCHOR_COUNT > 0) {
      numActive = locoDeckGetActiveAnchorIdList(anchorList, PREFLIGHT_MIN_LOCO_ANCHOR_COUNT);
    } else {
      numActive = 0;
    }

    PASS_IF_AND_ONLY_IF(numActive >= PREFLIGHT_MIN_ACTIVE_LOCO_ANCHOR_COUNT);
  } else if (PREFLIGHT_MIN_LH_BS_COUNT > 0) {
    /* Positioning system is Lighthouse */
    unsigned int baseStationsWithCalibrationAndGeometryData = (
      logGetUint(logIds.lighthouseBsHasCalibrationData) &
      logGetUint(logIds.lighthouseBsHasGeometryData)
    );
    unsigned int activeBaseStations = logGetUint(logIds.lighthouseBsActive);
    uint8_t i, mask = 1;

    for (i = 0; i < CONFIG_DECK_LIGHTHOUSE_MAX_N_BS; i++) {
      if (baseStationsWithCalibrationAndGeometryData & mask) {
        numEnabled++;

        if (activeBaseStations & mask) {
          numActive++;
        }
      }

      mask <<= 1;
    }

    PASS_IF_AND_ONLY_IF(
      numEnabled >= PREFLIGHT_MIN_LH_BS_COUNT &&
      numActive >= PREFLIGHT_MIN_ACTIVE_LH_BS_COUNT
    );
  } else if (PREFLIGHT_CHECK_ACTIVE_MARKER_DECK) {
    /* Positioning system is using active markers, check whether the active
     * marker deck is connected */
    PASS_IF_AND_ONLY_IF(paramGetUint(paramIds.activeMarkerDeckConnected));
  } else {
    SKIP;
  }
}

/**
 * Preflight check that determines whether the sensors have been initialized
 * successfully.
 */
static preflight_check_result_t testSensors() {
  PASS_IF_AND_ONLY_IF(sensorsAreCalibrated());
}

/**
 * Preflight check that tests whether a trajectory is defined in the high
 * level commander.
 */
static preflight_check_result_t testTrajectoriesAndLightsAreDefined() {
  uint8_t i;

  if (PREFLIGHT_MIN_TRAJECTORIES <= 0 && PREFLIGHT_MIN_LIGHT_PROGRAMS <= 0) {
    SKIP;
  } else {
    for (i = 0; i < PREFLIGHT_MIN_TRAJECTORIES; i++) {
      FAIL_UNLESS(crtpCommanderHighLevelIsTrajectoryDefined(i));
    }

    for (i = 0; i < PREFLIGHT_MIN_LIGHT_PROGRAMS; i++) {
      FAIL_UNLESS(lightProgramPlayerIsProgramDefined(i));
    }

    PASS;
  }
}

/**
 * Preflight check that tests whether the internal supervisor is running and
 * is in a state that allows takeoff.
 */
static preflight_check_result_t testSupervisor() {
  FAIL_UNLESS(supervisorIsArmed() || supervisorCanArm());
  PASS;
}

/**
 * Timer function that is called regularly (2 times every second by default).
 * This function executes the preflight checks and sets the corresponding log
 * variable appropriately.
 */
static void preflightTimer(xTimerHandle timer) {
  workerSchedule(preflightWorker, NULL);
}

/**
 * Worker function that is called regularly on the worker thread. This
 * function is responsible for executing all the preflight checks and for
 * updating the preflight check status and summary.
 */
static void preflightWorker(void* data) {
  uint16_t newStatus = 0;
  uint8_t index = 0;

#define CHECK_START(NAME) {}
#define CHECK_END(NAME, RESULT) {                     \
  newStatus |= (((RESULT) & 0x03) << (index * 2));    \
  index++;                                            \
}
#define SKIP_CHECK(NAME) {                            \
  CHECK_START(NAME);                                  \
  CHECK_END(NAME, preflightResultOff);                \
}
#define RUN_CHECK(NAME, EXPR) {                       \
  CHECK_START(NAME);                                  \
  CHECK_END(NAME, isForcedToPass ? preflightResultPass : (EXPR)); \
}

  /* IMPORTANT: the order of the tests must be in exactly the same way we
   * want to put them in the result variable, from right (LSB) to left (MSB).
   * Conceptually, we go from simpler, low-level tests (such as battery
   * voltage or the state of the stabilizer) to high-level tests (such as
   * the positioning system or the uploaded trajectory), except where backward
   * compatibility with earlier versions requires us to do otherwise.
   *
   * The order of the tests must also pass the order in the preflight_check_t
   * enum.
   */

  if (isEnabled) {
    RUN_CHECK(battery, testBattery());
    RUN_CHECK(sensors, testSensors());
    RUN_CHECK(kalmanFilter, testKalmanFilter());
    RUN_CHECK(positioning, testPositioningSystem());
    RUN_CHECK(home, testHomePosition());
    RUN_CHECK(trajectory, testTrajectoriesAndLightsAreDefined());
    RUN_CHECK(internal, testSupervisor());
  }

#undef CHECK_START
#undef CHECK_END
#undef SKIP_CHECK
#undef RUN_CHECK

  /* Store the new result of the preflight check and update the summary. We
   * assume that these variables are not updated from elsewhere so we don't
   * need to protect them with a semaphore */
  preflightCheckStatus = newStatus;
  preflightCheckSummary = calculatePreflightCheckSummary(preflightCheckStatus);
}

PARAM_GROUP_START(preflight)
PARAM_ADD(PARAM_UINT8, enabled, &isEnabled)
PARAM_ADD(PARAM_UINT8, force, &isForcedToPass)
PARAM_ADD(PARAM_FLOAT, homeX, &homeCoordinate[0])
PARAM_ADD(PARAM_FLOAT, homeY, &homeCoordinate[1])
PARAM_ADD(PARAM_FLOAT, homeZ, &homeCoordinate[2])
PARAM_GROUP_STOP(preflight)

LOG_GROUP_START(preflight)
LOG_ADD(LOG_UINT16, status, &preflightCheckStatus)
LOG_ADD(LOG_UINT8, summary, &preflightCheckSummary)
LOG_GROUP_STOP(preflight)
