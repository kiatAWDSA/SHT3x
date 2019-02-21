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

#include <I2C.h>

// Statuses/Errors returned by the functions in this class
typedef enum
{
  SHT3X_STATUS_OK = 0,  // No problemo.
  SHT3X_STATUS_FAIL = 1,  // Something went wrong.
  SHT3X_STATUS_NOTREADY = 2,  // The sensor is not ready to send out data.
  SHT3X_STATUS_CORRUPT = 3,   // Either the relative humidity or temperature bytes failed the CRC check.
  SHT3X_STATUS_DATA_LESS  = 4,  // Received less data bytes than expected.
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

  SHT3x(I2C * i2cWire);
  SHT3x(I2C * i2cWire, bool ADDRPinHigh);
  void changeAddress(bool ADDRPinHigh);
  SHT3X_STATUS triggerOneMeasurement(bool stretchClock, Repeatability repeatability);
  SHT3X_STATUS fetchMeasurement();
  double getRH();
  double getTemperature();

private:
  // The sensor has a "base" address that can be modified depending on the state of the ADDR pin (pin 2)
  static const uint8_t BASE_ADDRESS = 0x44;

  // Maximum duration (ms) needed to complete a measurement during the one-shot mode.
  // See datasheet Table 4.
  static const uint8_t DURATION_HIGREP = 15;
  static const uint8_t DURATION_MEDREP = 6;
  static const uint8_t DURATION_LOWREP = 4;

  // Number of bytes for I2C transmission
  static const uint8_t BYTECOUNT_DAQ_TOTAL  = 6;
  static const uint8_t BYTECOUNT_DAQ_TEMP   = 2;
  static const uint8_t BYTECOUNT_DAQ_RH     = 2;
  static const uint8_t BYTECOUNT_DAQ_CRC    = 1;

  /********************
   * DATA ACQUISITION *
   ********************/
  // One shot mode, clock stretching enabled
  static const uint8_t COM_DAQ_ONE_STRETCH_MSB       = 0x2C;
  static const uint8_t COM_DAQ_ONE_STRETCH_LSB_HIGREP   = 0x06;
  static const uint8_t COM_DAQ_ONE_STRETCH_LSB_MEDREP = 0x0D;
  static const uint8_t COM_DAQ_ONE_STRETCH_LSB_LOWREP = 0x10;

  // One shot mode, clock stretching disabled
  static const uint8_t COM_DAQ_ONE_NOSTRETCH_MSB       = 0x24;
  static const uint8_t COM_DAQ_ONE_NOSTRETCH_LSB_HIGREP = 0x00;
  static const uint8_t COM_DAQ_ONE_NOSTRETCH_LSB_MEDREP = 0x0B;
  static const uint8_t COM_DAQ_ONE_NOSTRETCH_LSB_LOWREP = 0x16;

  // TODO: Only coding for single-shot mode, for now.....
  /*
  // Continuous mode, 0.5 measurement per second
  static const uint8_t COM_DAQ_CON_HMPS_MSB = 0x20;
  static const uint8_t COM_DAQ_CON_HMPS_LSB_HIGREP = 0x32;
  static const uint8_t COM_DAQ_CON_HMPS_LSB_MEDREP = 0x24;
  static const uint8_t COM_DAQ_CON_HMPS_LSB_LOWREP = 0x2F;

  // Continuous mode, 1 measurement per second
  static const uint8_t COM_DAQ_CON_1MPS_MSB = 0x21;
  static const uint8_t COM_DAQ_CON_1MPS_LSB_HIGREP = 0x30;
  static const uint8_t COM_DAQ_CON_1MPS_LSB_MEDREP = 0x26;
  static const uint8_t COM_DAQ_CON_1MPS_LSB_LOWREP = 0x2D;

  // Continuous mode, 2 measurements per second
  static const uint8_t COM_DAQ_CON_2MPS_MSB = 0x22;
  static const uint8_t COM_DAQ_CON_2MPS_LSB_HIGREP = 0x36;
  static const uint8_t COM_DAQ_CON_2MPS_LSB_MEDREP = 0x20;
  static const uint8_t COM_DAQ_CON_2MPS_LSB_LOWREP = 0x2B;

  // Continuous mode, 4 measurements per second
  static const uint8_t COM_DAQ_CON_4MPS_MSB = 0x23;
  static const uint8_t COM_DAQ_CON_4MPS_LSB_HIGREP = 0x34;
  static const uint8_t COM_DAQ_CON_4MPS_LSB_MEDREP = 0x22;
  static const uint8_t COM_DAQ_CON_4MPS_LSB_LOWREP = 0x29;

  // Continuous mode, 10 measurements per second
  static const uint8_t COM_DAQ_CON_10MPS_MSB = 0x27;
  static const uint8_t COM_DAQ_CON_10MPS_LSB_HIGREP = 0x37;
  static const uint8_t COM_DAQ_CON_10MPS_LSB_MEDREP = 0x21;
  static const uint8_t COM_DAQ_CON_10MPS_LSB_LOWREP = 0x2A;
  */

  I2C *i2cWire_;
  uint8_t i2cAddress_;
  double relativeHumidity_;
  double temperature_;

  bool checkCRC(const uint8_t *data, uint8_t len, uint8_t oriCRC);
};


#endif