/**
 * INO Trident Marlin Library:
 *
 *
 * Modifications by Kagan Ozten (@KOlasmics)
 * Copyright (c) 2023, Plasmics GmbH
 * All rights reserved.
 */

#include "../inc/MarlinConfig.h"

#include "InoTrident.h"

#if HAS_INO_TRIDENT
  #ifndef MAX31865_MIN_SAMPLING_TIME_MSEC
    #define MAX31865_MIN_SAMPLING_TIME_MSEC 0
#endif

#define DEBUG_OUT ENABLED(DEBUG_MAX31865)
#include "../core/debug_out.h"

// The maximum speed the MAX31865 can do is 5 MHz
SPISettings InoTrident::spiConfig = SPISettings(
  TERN(TARGET_LPC1768, SPI_EIGHTH_SPEED, TERN(ARDUINO_ARCH_STM32, SPI_CLOCK_DIV16, 200000)), // SPI_CLOCK_DIV4, 500000
  MSBFIRST,
  SPI_MODE1 // CPOL0 CPHA1
);

#if DISABLED(LARGE_PINMAP)

  /**
   * Create the interface object using software (bitbang) SPI for PIN values
   * less than or equal to 127.
   *
   * @param spi_cs    the SPI CS pin to use
   * @param spi_mosi  the SPI MOSI pin to use
   * @param spi_miso  the SPI MISO pin to use
   * @param spi_clk   the SPI clock pin to use
  */
  InoTrident::InoTrident(int8_t spi_cs, int8_t spi_mosi, int8_t spi_miso, int8_t spi_clk) {
    cselPin = spi_cs;
    mosiPin = spi_mosi;
    misoPin = spi_miso;
    sclkPin = spi_clk;
  }

  /**
   * Create the interface object using hardware SPI for PIN for PIN values less
   * than or equal to 127.
   *
   * @param spi_cs  the SPI CS pin to use along with the default SPI device
   */
  InoTrident::InoTrident(int8_t spi_cs) {
    cselPin = spi_cs;
    sclkPin = misoPin = mosiPin = -1;
  }

#else // LARGE_PINMAP

  /**
   * Create the interface object using software (bitbang) SPI for PIN values
   * which are larger than 127. If you have PIN values less than or equal to
   * 127 use the other call for SW SPI.
   *
   * @param spi_cs       the SPI CS pin to use
   * @param spi_mosi     the SPI MOSI pin to use
   * @param spi_miso     the SPI MISO pin to use
   * @param spi_clk      the SPI clock pin to use
   * @param pin_mapping  set to 1 for positive pin values
   */
  MAX31865::MAX31865(uint32_t spi_cs, uint32_t spi_mosi, uint32_t spi_miso, uint32_t spi_clk, uint8_t pin_mapping) {
    cselPin = spi_cs;
    mosiPin = spi_mosi;
    misoPin = spi_miso;
    sclkPin = spi_clk;
  }

  /**
   * Create the interface object using hardware SPI for PIN values which are
   * larger than 127. If you have PIN values less than or equal to 127 use
   * the other call for HW SPI.
   *
   * @param spi_cs       the SPI CS pin to use along with the default SPI device
   * @param pin_mapping  set to 1 for positive pin values
   */
  MAX31865::MAX31865(uint32_t spi_cs, uint8_t pin_mapping) {
    cselPin = spi_cs;
    sclkPin = misoPin = mosiPin = -1UL;  //-1UL or 0xFFFFFFFF or 4294967295
  }

#endif // LARGE_PINMAP

/**
 *
 * Instance & Class methods
 *
 */

/**
 * Initialize the SPI interface and set the number of RTD wires used
 *
 * @param wires     The number of wires as an enum: MAX31865_2WIRE, MAX31865_3WIRE, or MAX31865_4WIRE.
 * @param zero_res  The resistance of the RTD at 0°C, in ohms.
 * @param ref_res   The resistance of the reference resistor, in ohms.
 * @param wire_res  The resistance of the wire connecting the sensor to the RTD, in ohms.
 */
void InoTrident::begin() {

  pinMode(cselPin, OUTPUT);
  digitalWrite(cselPin, HIGH);

  DELAY_US(1000);

  DEBUG_ECHOLNPGM("Init INO Hardware SPI");                // INO relies on hardware SPI
  SPI.begin();    // Start and configure hardware SPI


  //DEBUG_ECHOLNPGM(
  //  TERN(LARGE_PINMAP, "LARGE_PINMAP", "Regular")
  //  " begin call with cselPin: ", cselPin,
  //  " misoPin: ", misoPin,
  //  " sclkPin: ", sclkPin,
  //  " mosiPin: ", mosiPin,
  //  " config: ", readRegister8(MAX31865_CONFIG_REG)
  //  );

  DELAY_US(1000);

  //updateInoPID(kP, kI, kD);

  resetErrorConfigReg();
}

/**
 * Return and clear the last fault value
 *
 * @return The raw unsigned 8-bit FAULT status register or spike fault
 */
uint8_t InoTrident::readFault() {
  uint8_t r = lastFault;
  lastFault = 0;
  return r;
}

/**
 * Clear last fault
 */
//void InoTrident::clearFault() {
  //setConfig(MAX31865_CONFIG_FAULTSTAT, 1);
//}

/**
 * Reset flags
 */
//void InoTrident::resetFlags() {
  //writeRegister8(MAX31865_CONFIG_REG, stdFlags);
//}

/**
 * Enable the bias voltage on the RTD sensor
 */
//void InoTrident::enableBias() {
//  //setConfig(MAX31865_CONFIG_BIAS, 1);
//}

/**
 * Start a one-shot temperature reading.
 */
void InoTrident::keepAlive() {
  //setConfig(MAX31865_CONFIG_1SHOT | MAX31865_CONFIG_BIAS, 1);
  readTempActualRaw();
}

//void InoTrident::runAutoFaultDetectionCycle() {
//  
//}

/**
 * Initialize standard flags with flags that will not change during operation (Hz, polling mode and no. of wires)
 *
 * @param wires The number of wires in enum format
 */
//void InoTrident::initFixedFlags(inotrident_numwires_t wires) {

  // set config-defined flags (same for all sensors)
  //stdFlags = TERN(MAX31865_50HZ_FILTER, MAX31865_CONFIG_FILT50HZ, MAX31865_CONFIG_FILT60HZ) |
  //           TERN(MAX31865_USE_AUTO_MODE, MAX31865_CONFIG_MODEAUTO | MAX31865_CONFIG_BIAS, MAX31865_CONFIG_MODEOFF);

  //if (wires == inotrident_3WIRE)
  //  stdFlags |= MAX31865_CONFIG_3WIRE;   // 3 wire
  //else
  //  stdFlags &= ~MAX31865_CONFIG_3WIRE;  // 2 or 4 wire
//}

/**
 * Read the raw 16-bit value from the RTD_REG in one shot mode. This will include
 * the fault bit, D0.
 *
 * @return The raw unsigned 16-bit register value with ERROR bit attached, NOT temperature!
 */
uint16_t InoTrident::readRaw() {

  return lastRead; //lastRead;
}

/**
 * Read the raw 16-bit value from the RTD_REG in one shot mode. This will include
 * the fault bit, D0.
 *
 * @return The raw unsigned 16-bit register value with ERROR bit attached, NOT temperature!
 */
uint16_t InoTrident::readTempActualRaw() {
  uint8_t rcv_bytes[3] = { 1, 2, 3 };

  // ### Write Reg Sequence ###
  DELAY_US(2000);
  digitalWrite(cselPin, LOW);
  DELAY_NS(500);
  SPI.beginTransaction(spiConfig);
  DELAY_US(1);

  SPI.transfer(0b00000010);
  DELAY_NS(500);
  SPI.transfer(0x00);
  DELAY_NS(500);
  SPI.transfer(0x00);
  DELAY_NS(500);

  SPI.endTransaction();
  DELAY_US(1);

  digitalWrite(cselPin, HIGH);
  DELAY_US(2000);

  // ### Read Reg Sequence ###
  digitalWrite(cselPin, LOW);
  DELAY_NS(500);
  SPI.beginTransaction(spiConfig);
  DELAY_US(1);

  rcv_bytes[0] = SPI.transfer(0b00000000);
  DELAY_NS(500);
  rcv_bytes[1] = SPI.transfer(0x00);
  DELAY_NS(500);
  rcv_bytes[2] = SPI.transfer(0x00);
  DELAY_NS(500);

  SPI.endTransaction();
  DELAY_US(1);

  digitalWrite(cselPin, HIGH);
  
  if(rcv_bytes[0] == 0b00000010){ // This will solve bit shift error to an extend
    uint16_t temp_buf = 0;
    temp_buf |= rcv_bytes[1];
    temp_buf = (temp_buf << 8) | rcv_bytes[2];

    if(temp_buf != 0){ // Temperature cannot be 0C, otherwise discard it
      lastRead = temp_buf;
      spiFailComm = 0; // After succesful read zero the counter
    }
  }else{
    spiFailComm++;
    SERIAL_ECHOLNPGM("INO SPI ERROR (", spiFailComm, ")");
  }

  if(spiFailComm > 2){ // There is a problem with comm definetly
    lastRead = 0b00010000; // show it as a heartbeat error
  }

  DELAY_US(2000);

  //SERIAL_ECHOLNPGM("ino lastread: (", lastRead, ")");

  return lastRead;
}

/**
 * Calculate the temperature in C from the RTD resistance.
 *
 * @param    rtd_res  the resistance value in ohms
 * @return            the temperature in °C
 */
float InoTrident::tempRead() {

  float temp = 0;

  temp = ((float)((lastRead >> 8) & 0x7F) * 16) + ((float)(lastRead & 0xFC) / 16);

  //SERIAL_ECHOLNPGM("ino temp: (", temp, ")"); // For debugging

  if(temp == 0){
    temp = 33.3; 
  }
  
  return temp;
}

void InoTrident::setTargetTemperature(celsius_t targettemp){
  uint16_t target_temp = targettemp;

  if (old_target_temp != target_temp){ // Only send target temp when it's changed
    old_target_temp = target_temp;

    //Write Reg Sequence
    DELAY_US(3000);
    digitalWrite(cselPin, LOW);
    DELAY_NS(500);
    SPI.beginTransaction(spiConfig);
    DELAY_US(1);

    SPI.transfer(0b00000101);
    //digitalWrite(mosiPin, HIGH);
    DELAY_NS(500);

    SPI.transfer(target_temp>>8);
    //digitalWrite(mosiPin, HIGH);
    DELAY_NS(500);

    SPI.transfer(target_temp);
    DELAY_NS(500);

    SPI.endTransaction();
    DELAY_US(1);

    digitalWrite(cselPin, HIGH);
    DELAY_US(3000);
  }

}

uint16_t InoTrident::readErrorConfigReg(){
    //Error + config REG READ
    uint8_t rcv_bytes[3] = { 1, 2, 3 };
    uint16_t err_cfg_buf = 0;

    //Write Reg Sequence
    DELAY_US(1000);
    digitalWrite(cselPin, LOW);
    DELAY_NS(500);

    SPI.beginTransaction(spiConfig);
    DELAY_NS(800);

    SPI.transfer(0b00001000);
    //digitalWrite(mosiPin, HIGH);
    DELAY_NS(500);
    SPI.transfer(0x00);
    //digitalWrite(mosiPin, HIGH);
    DELAY_NS(500);
    SPI.transfer(0x00);
    DELAY_NS(500);

    SPI.endTransaction();
    DELAY_NS(800);

    digitalWrite(cselPin, HIGH);
    DELAY_US(2000);

    digitalWrite(cselPin, LOW);
    DELAY_NS(500);

    SPI.beginTransaction(spiConfig);
    DELAY_NS(800);

    rcv_bytes[0] = SPI.transfer(0b00000000);
    //digitalWrite(mosiPin, HIGH);
    DELAY_NS(500);
    rcv_bytes[1] = SPI.transfer(0x00);
    //digitalWrite(mosiPin, HIGH);
    DELAY_NS(500);
    rcv_bytes[2] = SPI.transfer(0x00);
    DELAY_NS(500);
    SPI.endTransaction();
    DELAY_NS(800);

    digitalWrite(cselPin, HIGH);
    DELAY_US(2000);

    err_cfg_buf |= rcv_bytes[1];
    err_cfg_buf = (err_cfg_buf << 8) | rcv_bytes[2];

    return err_cfg_buf;
}

void InoTrident::resetErrorConfigReg(){
    //Error + config REG RESET
    DELAY_US(1000);

    digitalWrite(cselPin, LOW);
    DELAY_NS(300);

    SPI.beginTransaction(spiConfig);
    DELAY_NS(500);

    SPI.transfer(0b00001001);
    //digitalWrite(mosiPin, HIGH);
    DELAY_NS(300);
    SPI.transfer(0xFF);
    //digitalWrite(mosiPin, HIGH);
    DELAY_NS(300);
    SPI.transfer(0xFF);
    DELAY_NS(300);

    SPI.endTransaction();
    DELAY_NS(500);

    digitalWrite(cselPin, HIGH);
    DELAY_US(2000);
}

void InoTrident::updateInoPID(float P, float I, float D){
	uint32_t pid_buf = 0;
	uint8_t pid_buf_lsb = 0;
	uint8_t pid_buf_msb = 0;

  // Set the global ino variables 
  kP = P; 
  kI = I;
  kD = D;

	pid_buf = P * 100;
	pid_buf_lsb |= pid_buf;
	pid_buf_msb |= pid_buf >> 8;

	///////////////////// write kP
  DELAY_US(1000);
  digitalWrite(cselPin, LOW);
  DELAY_NS(300);

  SPI.beginTransaction(spiConfig);
  DELAY_NS(500);
  SPI.transfer(0b00010001);
  //digitalWrite(mosiPin, HIGH);
  DELAY_NS(300);
  SPI.transfer(pid_buf_msb);
  //digitalWrite(mosiPin, HIGH);
  DELAY_NS(300);
  SPI.transfer(pid_buf_lsb);
  DELAY_NS(300);

  SPI.endTransaction();
  DELAY_NS(500);

  digitalWrite(cselPin, HIGH);
  DELAY_US(2000);

	pid_buf = I * 100;
	pid_buf_lsb = 0;
	pid_buf_msb = 0;
	pid_buf_lsb |= pid_buf;
	pid_buf_msb |= pid_buf >> 8;

	///////////////////// write kI
  digitalWrite(cselPin, LOW);
  DELAY_NS(300);

  SPI.beginTransaction(spiConfig);
  DELAY_NS(500);

  SPI.transfer(0b00100001);
  //digitalWrite(mosiPin, HIGH);
  DELAY_NS(300);
  SPI.transfer(pid_buf_msb);
  //digitalWrite(mosiPin, HIGH);
  DELAY_NS(300);
  SPI.transfer(pid_buf_lsb);
  DELAY_NS(300);

  SPI.endTransaction();
  DELAY_NS(500);

  digitalWrite(cselPin, HIGH);
  DELAY_US(2000);

	pid_buf = D * 100;
	pid_buf_lsb = 0;
	pid_buf_msb = 0;
	pid_buf_lsb |= pid_buf;
	pid_buf_msb |= pid_buf >> 8;

	///////////////////// write kD
  digitalWrite(cselPin, LOW);
  DELAY_NS(300);

  SPI.beginTransaction(spiConfig);
  DELAY_NS(500);

  SPI.transfer(0b01000001);
  //digitalWrite(mosiPin, HIGH);
  DELAY_NS(300);
  SPI.transfer(pid_buf_msb);
  //digitalWrite(mosiPin, HIGH);
  DELAY_NS(300);
  SPI.transfer(pid_buf_lsb);
  DELAY_NS(300);

  SPI.endTransaction();
  DELAY_NS(500);

  digitalWrite(cselPin, HIGH);
  DELAY_US(2000);

}


void InoTrident::softSpiInit() {
  DEBUG_ECHOLNPGM("Initializing MAX31865 Software SPI");
  pinMode(sclkPin, OUTPUT);
  digitalWrite(sclkPin, LOW);
  pinMode(mosiPin, OUTPUT);
  pinMode(misoPin, INPUT);
}

#endif // HAS_MAX31865 && !USE_ADAFRUIT_MAX31865
