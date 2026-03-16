/*
 * This file is part of INAV Project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "drivers/time.h"
#include "sensors/esc_sensor.h"

#define SRXL2_ESC_UPDATE_HZ 100

bool srxl2EscInit(void);
void srxl2EscWriteMotor(uint8_t index, uint16_t value);
void srxl2EscUpdate(timeUs_t currentTimeUs);
bool srxl2EscIsInitialized(void);
bool srxl2EscGetTelemetry(uint8_t index, escSensorData_t *data);