/*********************************************************************************
Arduino Library for the Sensirion SHT3x relative humidity and temperature sensor.
Copyright (C) 2019 Soon Kiat Lau

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*********************************************************************************/

#ifndef _SHT3X_h
#define _SHT3X_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <EEPROM.h>
#include <I2C.h>

// Statuses/Errors returned by the functions in this class
typedef enum
{
  SHT3X_STATUS_OK         = 0,  // No problemo.
  SHT3X_STATUS_FAIL       = 1,  // Something went wrong.
  SHT3X_STATUS_NOTREADY   = 2,  // The sensor is not ready to send out data.
  SHT3X_STATUS_CORRUPT    = 3,  // Either the relative humidity or temperature bytes failed the CRC check.
  SHT3X_STATUS_DATA_LESS  = 4   // Received less data bytes than expected.
} SHT3X_STATUS;



class SHT3x
{
public:
  // Repeatability of measurements
  typedef enum
  {
    REP_LOW = 0,
    REP_MED = 1,
    REP_HIG = 2
  } Repeatability;

  // Maximum duration (ms) needed to complete a measurement during the one-shot mode.
  // See datasheet Table 4.
  static const uint8_t DURATION_HIGREP = 15;
  static const uint8_t DURATION_MEDREP = 6;
  static const uint8_t DURATION_LOWREP = 4;

  SHT3x(I2C * i2cWire);
  SHT3x(I2C * i2cWire, bool ADDRPinHigh);
  void changeAddress(bool ADDRPinHigh);
  SHT3X_STATUS triggerOneMeasurement(bool stretchClock, Repeatability repeatability);
  SHT3X_STATUS fetchMeasurement();
  double getRH();
  double getRHRaw();
  double getTemperature();
  bool getSavedCalibration(bool point1, float * RHOutputRef, float * RHOutputRaw);
  void saveAndApplyCalibration(bool calPoint1, float RHRef, float RHRaw);
  void resetCalibration();

private:
  // The sensor has a "base" address that can be modified depending on the state of the ADDR pin (pin 2)
  const uint8_t BASE_ADDRESS = 0x44;

  // EEPROM storage locations for two-point calibration data.
  // Note that float values (ref and raw RH) take up 4 bytes.
  const uint8_t EEPROM_ADDR_POINT1_CRC    = 10;
  const uint8_t EEPROM_ADDR_POINT1_RHREF  = 12;
  const uint8_t EEPROM_ADDR_POINT1_RHRAW  = 16;
  const uint8_t EEPROM_ADDR_POINT2_CRC    = 20;
  const uint8_t EEPROM_ADDR_POINT2_RHREF  = 22;
  const uint8_t EEPROM_ADDR_POINT2_RHRAW  = 26;

  // Default values for two-point calibration
  const float RH_POINT1_DEFAULT = 1;
  const float RH_POINT2_DEFAULT = 100;

  // Number of bytes for I2C transmission
  static const uint8_t BYTECOUNT_DAQ_TOTAL  = 6;
  static const uint8_t BYTECOUNT_DAQ_TEMP   = 2;
  static const uint8_t BYTECOUNT_DAQ_RH     = 2;
  static const uint8_t BYTECOUNT_DAQ_CRC    = 1;

  // Data buffers
  uint8_t dataBuffer[BYTECOUNT_DAQ_TOTAL];
  uint8_t tempBuffer[BYTECOUNT_DAQ_TEMP];
  uint8_t RHBuffer[BYTECOUNT_DAQ_RH];

  /********************
   * DATA ACQUISITION *
   ********************/
  // One shot mode, clock stretching enabled
  const uint8_t COM_DAQ_ONE_STRETCH_MSB         = 0x2C;
  const uint8_t COM_DAQ_ONE_STRETCH_LSB_HIGREP  = 0x06;
  const uint8_t COM_DAQ_ONE_STRETCH_LSB_MEDREP  = 0x0D;
  const uint8_t COM_DAQ_ONE_STRETCH_LSB_LOWREP  = 0x10;

  // One shot mode, clock stretching disabled
  const uint8_t COM_DAQ_ONE_NOSTRETCH_MSB         = 0x24;
  const uint8_t COM_DAQ_ONE_NOSTRETCH_LSB_HIGREP  = 0x00;
  const uint8_t COM_DAQ_ONE_NOSTRETCH_LSB_MEDREP  = 0x0B;
  const uint8_t COM_DAQ_ONE_NOSTRETCH_LSB_LOWREP  = 0x16;

  // TODO: Only coding for single-shot mode, for now.....
  /*
  // Continuous mode, 0.5 measurement per second
  const uint8_t COM_DAQ_CON_HMPS_MSB = 0x20;
  const uint8_t COM_DAQ_CON_HMPS_LSB_HIGREP = 0x32;
  const uint8_t COM_DAQ_CON_HMPS_LSB_MEDREP = 0x24;
  const uint8_t COM_DAQ_CON_HMPS_LSB_LOWREP = 0x2F;

  // Continuous mode, 1 measurement per second
  const uint8_t COM_DAQ_CON_1MPS_MSB = 0x21;
  const uint8_t COM_DAQ_CON_1MPS_LSB_HIGREP = 0x30;
  const uint8_t COM_DAQ_CON_1MPS_LSB_MEDREP = 0x26;
  const uint8_t COM_DAQ_CON_1MPS_LSB_LOWREP = 0x2D;

  // Continuous mode, 2 measurements per second
  const uint8_t COM_DAQ_CON_2MPS_MSB = 0x22;
  const uint8_t COM_DAQ_CON_2MPS_LSB_HIGREP = 0x36;
  const uint8_t COM_DAQ_CON_2MPS_LSB_MEDREP = 0x20;
  const uint8_t COM_DAQ_CON_2MPS_LSB_LOWREP = 0x2B;

  // Continuous mode, 4 measurements per second
  const uint8_t COM_DAQ_CON_4MPS_MSB = 0x23;
  const uint8_t COM_DAQ_CON_4MPS_LSB_HIGREP = 0x34;
  const uint8_t COM_DAQ_CON_4MPS_LSB_MEDREP = 0x22;
  const uint8_t COM_DAQ_CON_4MPS_LSB_LOWREP = 0x29;

  // Continuous mode, 10 measurements per second
  const uint8_t COM_DAQ_CON_10MPS_MSB = 0x27;
  const uint8_t COM_DAQ_CON_10MPS_LSB_HIGREP = 0x37;
  const uint8_t COM_DAQ_CON_10MPS_LSB_MEDREP = 0x21;
  const uint8_t COM_DAQ_CON_10MPS_LSB_LOWREP = 0x2A;
  */

  I2C *i2cWire_;
  uint8_t i2cAddress_;
  double relativeHumidity_;
  double temperature_;
  float slopeAdjustment_;
  float offset_;

  uint8_t calcCRC(const uint8_t *data, uint8_t len);
  uint8_t calcCRCRefAndRaw(float RHRef, float RHRaw);
  void calcRHAdj();
};


#endif