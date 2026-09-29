/*
 * Crazyflie GCS-controlled light effect evaluation module
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

#include "gcs_light_effects.h"

#include <errno.h>
#include <string.h>

#define DEBUG_MODULE "GCS_FX"
#include "debug.h"

static bool isInit = false;

/** Current light effect */
static gcs_light_effect_type_t currentEffect = GCS_LIGHT_EFFECT_OFF;

/** Current color associated to the light effect */
static uint8_t currentColor[3] = { 255, 255, 255 };

void gcsLightEffectsInit(void) {
  if (isInit) {
    return;
  }

  isInit = true;
}

bool gcsLightEffectsTest(void) {
  return isInit;
}

void gcsLightEffectDisable() {
  gcsLightEffectTrigger(GCS_LIGHT_EFFECT_OFF, 0);
}

void gcsLightEffectEvaluate(uint8_t* color) {
  switch (currentEffect) {
    case GCS_LIGHT_EFFECT_SOLID:
      memcpy(color, currentColor, 3);
      break;

    default:
      memset(color, 0, 3);
  }
}

int gcsLightEffectTrigger(gcs_light_effect_type_t effect, const uint8_t* color) {
  switch (effect) {
    case GCS_LIGHT_EFFECT_OFF:
      break;

    case GCS_LIGHT_EFFECT_SOLID:
      break;

    default:
      return ENOEXEC;
  }

  currentEffect = effect;
  if (color) {
    memcpy(currentColor, color, 3);
  }

  return 0;
}

bool areGcsLightEffectsActive(void) {
  return currentEffect != GCS_LIGHT_EFFECT_OFF;
}
