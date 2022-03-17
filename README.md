# X-NUCLEO-53L4A2

Arduino library to support the X-NUCLEO-53L4A2 based on VL53L4CX Time-of-Flight high accuracy ranging sensor with multi target detection.
This sensor uses I2C to communicate. An I2C instance is required to access to the sensor.
The APIs provide simple distance measure in both polling and interrupt modes.

## Examples

There are 2 examples with the X-NUCLEO-53L4A2 library.

* X_NUCLEO_53L4A2_HelloWorld: This example code is to show how to get proximity
  values of the onboard VL53L4CX sensor in polling mode.

* X_NUCLEO_53L4A2_HelloWorld_Interrupt: This example code is to show how to get proximity
  values of the onboard VL53L4CX sensor in interrupt mode.

## Dependencies

This package requires the following Arduino libraries:

* STM32duino VL53L4CX: https://github.com/stm32duino/VL53L4CX


## Documentation

You can find the source files at  
https://github.com/stm32duino/X-NUCLEO-53L4A2

The VL53L4CX datasheet is available at  
https://www.st.com/en/imaging-and-photonics-solutions/vl53l4cx.html