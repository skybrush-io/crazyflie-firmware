/*
 * Parameter updates required for the Skybrush compatibility layer
 *
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

#define DEBUG_MODULE "PARAMS"

#include "autoconf.h"

#include "debug.h"
#include "param_logic.h"

#include "custom_params.h"

typedef struct {
  const char* group;
  const char* name;
  float value;
  uint8_t flags;
} customParamTableEntry_t;

#define NO_MORE_ENTRIES { 0, 0, 0 }

#define TYPE_FLOAT 0
#define OPTIONAL 1
#define TYPE_INT 2

static const customParamTableEntry_t params[] = {
  /* Attitude rate controller PID tuning */
#ifdef CONFIG_PLATFORM_BOLT
  { "pid_rate", "roll_kp",   70 },
  { "pid_rate", "roll_ki",  200 },
  { "pid_rate", "roll_kd",    2 },
  { "pid_rate", "pitch_kp",  70 },
  { "pid_rate", "pitch_ki", 200 },
  { "pid_rate", "pitch_kd",   2 },
#else
  { "pid_rate", "roll_kp",  220 },
  { "pid_rate", "roll_ki",  500 },
  { "pid_rate", "roll_kd",    2 },
  { "pid_rate", "pitch_kp", 220 },
  { "pid_rate", "pitch_ki", 500 },
  { "pid_rate", "pitch_kd",   2 },
#endif

  /* Attitude controller PID tuning */
#ifdef CONFIG_PLATFORM_BOLT
  { "pid_attitude", "roll_kp",  7 },
  { "pid_attitude", "pitch_kp", 7 },
  { "pid_attitude", "roll_ki",  3 },
  { "pid_attitude", "pitch_ki", 3 },
#else
  { "pid_attitude", "roll_kp",  6 },
  { "pid_attitude", "pitch_kp", 6 },
  { "pid_attitude", "roll_ki",  0 },
  { "pid_attitude", "pitch_ki", 0 },
#endif

  /* Position controller PID tuning */
  { "posCtlPid", "xVelMax", 2, OPTIONAL },
  { "posCtlPid", "yVelMax", 2, OPTIONAL },
#ifdef CONFIG_PLATFORM_BOLT
  { "posCtlPid", "zVelMax", 1.5, OPTIONAL },
#else
  { "posCtlPid", "zVelMax", 0.8, OPTIONAL },
#endif
  { "posCtlPid", "vxKFF", 1, OPTIONAL },
  { "posCtlPid", "vyKFF", 1, OPTIONAL },

#ifdef CONFIG_PLATFORM_BOLT
  /* Larger Bolt-bases show drones are okay with a lower thrust base */
  { "posCtlPid", "thrustBase", 30000, TYPE_INT },

  /* Currently we always use single-cell batteries so the voltage thresholds
   * need to be adjusted */
  { "pm", "lowVoltage", 3.2 },
  { "pm", "criticalLowVoltage", 3.0 },
#endif

#ifdef CONFIG_SHOW_TDOA_LARGE_AREA
  { "tdoa3", "stddev", 0.8 },
#endif

#ifdef CONFIG_SHOW_STEALTH_MODE
  { "led", "bitmask", 128, TYPE_INT },
#endif

  NO_MORE_ENTRIES
};

bool droneShowApplyCustomParameters(void) {
  const customParamTableEntry_t* entry;
  bool allValid = true;

  for (entry = params; entry->group && entry->name; entry++) {
    paramVarId_t paramId = paramGetVarId(entry->group, entry->name);

    if (PARAM_VARID_IS_VALID(paramId)) {
      if (entry->flags & TYPE_INT) {
        if (paramGetInt(paramId) != entry->value) {
          paramSetInt(paramId, entry->value);
        }
      } else {
        if (paramGetFloat(paramId) != entry->value) {
          paramSetFloat(paramId, entry->value);
        }
      }
    } else if (!(entry->flags & OPTIONAL)) {
      if (allValid) {
        DEBUG_PRINT("Cannot set param %s.%s\n", entry->group, entry->name);
      }
      allValid = false;
    }
  }

  return allValid;
}
