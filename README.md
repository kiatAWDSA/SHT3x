# SHT3x
The Sensirion SHT3x relative humidity and temperature sensors communicate with the I2C protocol. Unfortunately, the in-built Wire library provided in the Arduino distribution has been known to block indefinitely in rare circumstances, which is undesirable for equipment that will be operating for a long time. This library utilizes a custom non-blocking I2C library to solve that problem.

Custom I2C library: https://github.com/kiatAWDSA/I2C

Blocking behavior of Wire libary (search for "blocking"):
 - https://playground.arduino.cc/Main/WireLibraryDetailedReference
 - https://arduino.stackexchange.com/a/30354

## Installation
TBD

## Integrating it into your sketch
TBD
