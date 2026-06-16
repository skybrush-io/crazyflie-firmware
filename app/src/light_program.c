/*
 * Crazyflie preprogrammed light pattern execution module
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

#include <errno.h>
#include <memory.h>

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "semphr.h"

/* libskybrush includes */
#include <skybrush/lights.h>

#include "light_program.h"
#include "mem.h"

#define DEBUG_MODULE "LIGHT"
#include "debug.h"

/**
 * Size of the memory segment that can store light programs.
 */
#define LIGHT_PROGRAM_MEMORY_SIZE 4096

#define NUM_LIGHT_PROGRAM_DEFINITIONS 10

uint8_t lightProgramMemory[LIGHT_PROGRAM_MEMORY_SIZE];
static struct lightProgramDescription lightProgramDescriptions[NUM_LIGHT_PROGRAM_DEFINITIONS];

static bool isInit = false;

// Light program memory handling from the memory module
static uint32_t handleMemGetSize(const uint8_t internal_id) { return lightProgramPlayerMemSize(); }
static bool handleMemRead(const uint8_t internal_id, const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer);
static bool handleMemWrite(const uint8_t internal_id, const uint32_t memAddr, const uint8_t writeLen, const uint8_t* buffer);
static const MemoryHandlerDef_t memDef = {
  .type = MEM_TYPE_APP,
  .getSize = handleMemGetSize,
  .read = handleMemRead,
  .write = handleMemWrite,
};

// makes sure that we don't evaluate the light program while it is being changed
static xSemaphoreHandle lockLightProgram;
static StaticSemaphore_t lockLightProgramBuffer;

static struct lightProgramDescription* currentProgram;   // the light program that we are playing
static uint64_t startedAt;         // timestamp when the current program started
static float currentTimescale = 1; // timescale of the current program being played

static sb_light_program_t lightSequence;
static bool lightSequenceInitialized = 0;

static sb_light_player_t lightSequencePlayer;  // player object that plays Skybrush light sequences

static uint8_t* startOfCurrentProgramInMemory();
static uint32_t sizeOfCurrentProgramInMemory();

static uint16_t timeToFrame(float t);
static uint8_t* timeToPointer(float t, uint8_t bytesPerFrame);

static void evaluateBlack(float t, uint8_t* color);
static void evaluateRGBAt(float t, uint8_t* color);
static void evaluateRGB565At(float t, uint8_t* color);
static void evaluateLightSequenceAt(float t, uint8_t* color);

typedef void (*evaluator_t)(float, uint8_t*);
static evaluator_t evaluator = evaluateBlack;

void lightProgramPlayerInit(void) {
  if (isInit) {
    return;
  }

  memoryRegisterHandler(&memDef);

  lockLightProgram = xSemaphoreCreateMutexStatic(&lockLightProgramBuffer);

  if (sb_light_program_init_empty(&lightSequence)) {
    isInit = false;
  } else if (sb_light_player_init(&lightSequencePlayer, &lightSequence)) {
    sb_light_program_destroy(&lightSequence);
    isInit = false;
  } else {
    lightSequenceInitialized = true;
    isInit = true;
  }
}

bool lightProgramPlayerTest(void) {
  return isInit;
}

uint32_t lightProgramPlayerMemSize() {
  return sizeof(lightProgramMemory);
}

int lightProgramPlayerDefineProgram(uint8_t programId, struct lightProgramDescription description) {
  if (programId >= NUM_LIGHT_PROGRAM_DEFINITIONS) {
    return ENOEXEC;
  }

  lightProgramDescriptions[programId] = description;

  return 0;
}

void lightProgramPlayerEvaluate(uint8_t* color) {
  uint64_t now;
  float t;

  xSemaphoreTake(lockLightProgram, portMAX_DELAY);

  now = usecTimestamp();
  t = (now < startedAt) ? 0 : ((now - startedAt) / 1e6);

  if (!currentProgram) {
    evaluator = evaluateBlack;
  }

  evaluator(t * currentTimescale, color);

  xSemaphoreGive(lockLightProgram);
}

void lightProgramPlayerEvaluateAt(float t, uint8_t* color) {
  xSemaphoreTake(lockLightProgram, portMAX_DELAY);

  if (!currentProgram) {
    evaluator = evaluateBlack;
  }

  evaluator(t * currentTimescale, color);

  xSemaphoreGive(lockLightProgram);
}

bool lightProgramPlayerIsProgramDefined(uint8_t programId) {
  return (
    programId < NUM_LIGHT_PROGRAM_DEFINITIONS &&
    lightProgramDescriptions[programId].lightProgramLocation != LIGHT_PROGRAM_LOCATION_INVALID
  );
}

static void evaluateBlack(float scaledT, uint8_t* color) {
  color[0] = color[1] = color[2] = 0;
}

static void evaluateRGBAt(float scaledT, uint8_t* color) {
  uint8_t* ptr = timeToPointer(scaledT, 3);
  if (!ptr) {
    return evaluateBlack(scaledT, color);
  } else {
    memcpy(color, ptr, 3);
  }
}

static void evaluateRGB565At(float scaledT, uint8_t* color) {
  uint8_t* ptr = timeToPointer(scaledT, 2);
  uint8_t x;

  if (!ptr) {
    return evaluateBlack(scaledT, color);
  } else {
    /* red */
    x = ptr[0] & 0xf8;
    color[0] = x | (x >> 5);

    /* green */
    x = ((ptr[0] & 0x07) << 3) | ((ptr[1] & 0xe0) >> 5);
    color[1] = (x << 2) | (x >> 4);

    /* blue */
    x = ptr[1] & 0x1f;
    color[2] = (x << 3) | (x >> 2);
  }
}

static void evaluateLightSequenceAt(float scaledT, uint8_t* color) {
  sb_rgb_color_t rgb_color = lightSequenceInitialized
    ? sb_light_player_get_color_at(&lightSequencePlayer, scaledT * 1000)
    : SB_COLOR_BLACK;

  color[0] = rgb_color.red;
  color[1] = rgb_color.green;
  color[2] = rgb_color.blue;
}

static bool handleMemRead(const uint8_t internal_id, const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer) {
  bool result = false;

  if (memAddr + readLen <= sizeof(lightProgramMemory)) {
    memcpy(buffer, &(lightProgramMemory[memAddr]), readLen);
    result = true;
  }

  return result;
}

static bool handleMemWrite(const uint8_t internal_id, const uint32_t memAddr, const uint8_t writeLen, const uint8_t* buffer) {
  bool result = false;

  if ((memAddr + writeLen) <= sizeof(lightProgramMemory)) {
    memcpy(&(lightProgramMemory[memAddr]), buffer, writeLen);
    result = true;
  }

  return result;
}

int lightProgramPlayerPlay(uint8_t programId, float timescale) {
  return lightProgramPlayerSchedulePlayFrom(usecTimestamp(), programId, timescale);
}

int lightProgramPlayerPause(void) {
  /* TODO(ntamas) */
  return ENOEXEC;
}

int lightProgramPlayerResume(void) {
  /* TODO(ntamas) */
  return ENOEXEC;
}

int lightProgramPlayerSchedulePlayFrom(uint64_t timestamp, uint8_t programId, float timescale) {
  struct lightProgramDescription* description;
  struct lightProgramDescription* oldProgram;
  int result = 0;

  if (programId >= NUM_LIGHT_PROGRAM_DEFINITIONS) {
    return ENOEXEC;
  }

  description = &lightProgramDescriptions[programId];

  if (description->lightProgramLocation != LIGHT_PROGRAM_LOCATION_MEM) {
    return ENOEXEC;
  }

  xSemaphoreTake(lockLightProgram, portMAX_DELAY);

  switch (description->lightProgramType) {
    case LIGHT_PROGRAM_TYPE_RGB:
      evaluator = evaluateRGBAt;
      break;

    case LIGHT_PROGRAM_TYPE_RGB565:
      evaluator = evaluateRGB565At;
      break;

    case LIGHT_PROGRAM_TYPE_LEDCTRL:
      oldProgram = currentProgram;
      currentProgram = description;
      if (sizeOfCurrentProgramInMemory() > 65535) {
        /* light program too large */
        DEBUG_PRINT("Program too large\n");
        result = ENOEXEC;
      } else {
        evaluator = evaluateLightSequenceAt;
        if (lightSequenceInitialized) {
          sb_light_player_destroy(&lightSequencePlayer);
          sb_light_program_destroy(&lightSequence);
          lightSequenceInitialized = false;
        }
        if (sb_light_program_init_from_buffer(
          &lightSequence, startOfCurrentProgramInMemory(),
          sizeOfCurrentProgramInMemory()
        )) {
          /* something happened while loading the program */
          result = ENOEXEC;
        } else if (sb_light_player_init(&lightSequencePlayer, &lightSequence)) {
          /* something happened while initializing the player */
          sb_light_program_destroy(&lightSequence);
          result = ENOEXEC;
        } else {
          lightSequenceInitialized = true;
        }
      }
      currentProgram = oldProgram;
      break;

    default:
      result = ENOEXEC;
  }

  if (result == 0) {
    startedAt = timestamp;
    currentProgram = description;
    currentTimescale = timescale;
  } else {
    evaluator = evaluateBlack;
  }

  xSemaphoreGive(lockLightProgram);

  return result;
}

int lightProgramPlayerStop(void) {
  currentProgram = 0;
  evaluator = evaluateBlack;
  return 0;
}

static uint16_t timeToFrame(float t) {
  return currentProgram ? t * currentProgram->fps : 0;
}

static uint32_t sizeOfCurrentProgramInMemory() {
  return currentProgram ? currentProgram->lightProgramIdentifier.mem.size : 0;
}

static uint8_t* startOfCurrentProgramInMemory() {
  return currentProgram ? &lightProgramMemory[currentProgram->lightProgramIdentifier.mem.offset] : 0;
}

static uint8_t* timeToPointer(float t, uint8_t bytesPerFrame) {
  uint32_t offset;
  uint8_t* ptr;

  if (!currentProgram) {
    return 0;
  }

  offset = timeToFrame(t) * bytesPerFrame;
  ptr = startOfCurrentProgramInMemory();

  if (ptr == 0 || offset > sizeOfCurrentProgramInMemory() - bytesPerFrame) {
    return 0;
  } else {
    return ptr + offset;
  }
}
