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

#include "SHT3x.h"

// Initialize with only the custom i2c class
SHT3x::SHT3x(I2C & i2cWire) : i2cWire_(&i2cWire)
{
  // Default address to the base address
  changeAddress(false);
}

// Initialize with custom i2c class and set the state of address pin
SHT3x::SHT3x(I2C & i2cWire, bool ADDRPinHigh) : i2cWire_(&i2cWire)
{
  changeAddress(ADDRPinHigh);
}

void SHT3x::changeAddress(bool ADDRPinHigh)
{
  if (ADDRPinHigh)
  {
    i2cAddress_ = BASE_ADDRESS + 1; // 0x45 or 69
  }
  else
  {
    i2cAddress_ = BASE_ADDRESS; // 0x44 or 68
  }
}

// Trigger the sensor to perform a single measurement.
SHT3X_STATUS SHT3x::triggerOneMeasurement(bool stretchClock, Repeatability repeatability)
{
  uint8_t stretchByte;
  uint8_t repeatabilityByte;

  if (stretchClock)
  {
    stretchByte = COM_DAQ_ONE_STRETCH_MSB;

    switch (repeatability)
    {
    case REP_LOW:
      repeatabilityByte = COM_DAQ_ONE_STRETCH_LSB_LOWREP;
      break;
    case REP_MED:
      repeatabilityByte = COM_DAQ_ONE_STRETCH_LSB_MEDREP;
      break;
    case REP_HIG:
    default:
      repeatabilityByte = COM_DAQ_ONE_STRETCH_LSB_HIGREP;
      break;
    }
  }
  else
  {
    stretchByte = COM_DAQ_ONE_NOSTRETCH_MSB;

    switch (repeatability)
    {
    case REP_LOW:
      repeatabilityByte = COM_DAQ_ONE_NOSTRETCH_LSB_LOWREP;
      break;
    case REP_MED:
      repeatabilityByte = COM_DAQ_ONE_NOSTRETCH_LSB_MEDREP;
      break;
    case REP_HIG:
    default:
      repeatabilityByte = COM_DAQ_ONE_NOSTRETCH_LSB_HIGREP;
      break;
    }
  }

  if (i2cWire_->write(i2cAddress_, stretchByte, repeatabilityByte) == I2C_STATUS_OK)
  {
    return SHT3X_STATUS_OK;
    // The calling code should now wait for a minimum duration (e.g. DURATION_HIGREP) depending on the repeatability setting before trying to grab the measurement
  }
  else
  { // Something went wrong.
    return SHT3X_STATUS_FAIL;
  }
}

// Attempts to fetch measurement (need to trigger one-shot first). If the sensor is still processing the measurement, this returns SHT3X_STATUS_NOTREADY.
// Obtained data is stored internally. Must call getRH() or getTemperature() to get the actual measurement values.
// TODO: clock stretching is not implemented yet.
SHT3X_STATUS SHT3x::fetchMeasurement()
{
  I2C_STATUS i2cStatus;
  i2cStatus = i2cWire_->read(i2cAddress_, BYTECOUNT_DAQ_TOTAL);

  if (i2cStatus = I2C_STATUS_OK)
  { // Begin processing the signals
    // Grab data from I2C buffer
    uint8_t dataBuffer[BYTECOUNT_DAQ_TOTAL];
    for (uint8_t i = 0; i < BYTECOUNT_DAQ_TOTAL; i++)
    {
      if (i2cWire_->available())
      {
        dataBuffer[i] = i2cWire_->receive();
      }
      else
      { // Did not receive the expected amount of bytes
        return SHT3X_STATUS_DATA_LESS;
      }
    }

    // Organize the data
    uint8_t tempBytes[BYTECOUNT_DAQ_TEMP] = { dataBuffer[BYTECOUNT_DAQ_TEMP-2], dataBuffer[BYTECOUNT_DAQ_TEMP-1] };
    uint8_t tempCRC = dataBuffer[BYTECOUNT_DAQ_TEMP];
    uint8_t RHBytes[BYTECOUNT_DAQ_RH] = { dataBuffer[BYTECOUNT_DAQ_TEMP + BYTECOUNT_DAQ_CRC + BYTECOUNT_DAQ_RH - 2], dataBuffer[BYTECOUNT_DAQ_TEMP + BYTECOUNT_DAQ_CRC + BYTECOUNT_DAQ_RH - 1] };
    uint8_t RHCRC = dataBuffer[BYTECOUNT_DAQ_TEMP + BYTECOUNT_DAQ_CRC + BYTECOUNT_DAQ_RH];

    // Perform CRC check on the RH and temperature bytes
    if (checkCRC(tempBytes, BYTECOUNT_DAQ_TEMP, tempCRC) && checkCRC(RHBytes, BYTECOUNT_DAQ_RH, RHCRC))
    { // CRC check success; move on to convert signal into actual readings.
      // Refer to datasheet section 4.13.
      // Calculate temperature (degC; we're not imperial heathens)
      uint16_t tempSignal = tempBytes[BYTECOUNT_DAQ_TEMP-2];
      tempSignal = tempSignal << 8;
      tempSignal |= tempBytes[BYTECOUNT_DAQ_TEMP-1];
      temperature_ = -45 + tempSignal * 0.0026703; // The multiplier is 175/(2^16 - 1), rounded to account for precision of float in Arduino

      // Calculate relative humidity
      uint16_t RHSignal = RHBytes[BYTECOUNT_DAQ_TEMP-2];
      RHSignal = RHSignal << 8;
      RHSignal |= RHBytes[BYTECOUNT_DAQ_TEMP-1];
      relativeHumidity_ = RHSignal * 0.0015259; // The multiplier is 100/(2^16 - 1), rounded to account for precision of float in Arduino

      return SHT3X_STATUS_OK;
    }
    else
    { // Failed CRC check for one of the readings
      return SHT3X_STATUS_CORRUPT;
    }
  }
  else if (i2cStatus == I2C_STATUS_ADDRESS_NACK)
  {
    // Received a NACK bit after sending out address byte, indicating the sensor is still performing the measurement.
    return SHT3X_STATUS_NOTREADY;
  }
  else
  { // Something went wrong.
    return SHT3X_STATUS_FAIL;
  }
}

double SHT3x::getRH()
{
  return relativeHumidity_;
}

double SHT3x::getTemperature()
{
  return temperature_;
}

// Calculate the CRC checksum of the data bytes.
// Adapted from the Arduino SHT library by Sensirion:
// https://github.com/Sensirion/arduino-sht
bool SHT3x::checkCRC(const uint8_t *data, uint8_t byteCount, uint8_t oriCRC)
{
  uint8_t crc = 0xFF;
  for (uint8_t byteCtr = 0; byteCtr < byteCount; ++byteCtr) {
    crc ^= data[byteCtr];
    for (uint8_t bit = 8; bit > 0; --bit) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0x31;
      }
      else {
        crc = (crc << 1);
      }
    }
  }

  return (crc == oriCRC);
}
