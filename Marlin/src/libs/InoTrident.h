/**
 * INO Trident Marlin Library:
 *
 *
 * Modifications by Kagan Ozten (@KOlasmics)
 * Copyright (c) 2023, Plasmics GmbH
 * All rights reserved.
 */
#pragma once

//#define DEBUG_MAX31865

#include "../inc/MarlinConfig.h"
#include "../HAL/shared/Delay.h"
#include HAL_PATH(../HAL, MarlinSPI.h)

typedef enum inotrident_numwires {
  inotrident_2WIRE = 0,
  inotrident_3WIRE = 1,
  inotrident_4WIRE = 0
} inotrident_numwires_t;

/* Interface class for the Ino Trident */
class InoTrident {
private:
  static SPISettings spiConfig;

  TERN(LARGE_PINMAP, uint32_t, uint8_t) sclkPin, misoPin, mosiPin, cselPin;

  uint16_t spiDelay;

  uint16_t lastRead = 0;
  uint8_t lastFault = 0;

  void softSpiInit();
  void runAutoFaultDetectionCycle();

  //INO
  uint16_t old_target_temp = 0;
  uint8_t spiFailComm = 0;


public:
  #if ENABLED(LARGE_PINMAP)
    MAX31865(uint32_t spi_cs, uint8_t pin_mapping);
    MAX31865(uint32_t spi_cs, uint32_t spi_mosi, uint32_t spi_miso,
             uint32_t spi_clk, uint8_t pin_mapping);
  #else
    InoTrident(int8_t spi_cs);
    InoTrident(int8_t spi_cs, int8_t spi_mosi, int8_t spi_miso,
             int8_t spi_clk);
  #endif

  void begin();
  void keepAlive();

  uint8_t hb_err_count = 0;

  uint8_t readFault();
  void clearFault();

  uint16_t readRaw();
  float readResistance();
  float tempRead();

  //INO Specific functions
  uint16_t readTempActualRaw();
  void setTargetTemperature(celsius_t targettemp);
  uint16_t readErrorConfigReg();
  void resetErrorConfigReg();
  void updateInoPID(float P, float I, float D);

  float kP = 13.33;
  float kI = 30.91;
  float kD = 5.78;

};
