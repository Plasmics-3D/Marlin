/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2021 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

#define BOARD_INFO_NAME "BTT OCTOPUS V1.1"

//
// Temperature Sensors
//
#if TEMP_SENSOR_0 == 20
  #define TEMP_0_PIN                        PF8   // PT100 Connector
#else
  #define TEMP_0_PIN                        PF4   // TH0
#endif

//INO pinouts
#define TEMP_0_SCK_PIN PA5
#define TEMP_0_MISO_PIN PA6
#define TEMP_0_MOSI_PIN PA7
#define TEMP_0_CS_PIN PD3 // PD3 -> E3 CS PIN for octopus

//INO pinouts
#define TEMP_1_SCK_PIN PA5
#define TEMP_1_MISO_PIN PA6
#define TEMP_1_MOSI_PIN PA7
#define TEMP_1_CS_PIN PB6 // PD3 -> E3 CS PIN for octopus

#include "pins_BTT_OCTOPUS_V1_common.h"
