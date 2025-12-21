# VESC-CAN-OLED-display
Simple display for VESC controllers, allowing for full control via CAN bus

The program is built arround the STM32F103CBT6 that uses many of it's functions:

- native CAN implementation thanks to the STM32_CAN library from pazi88: https://github.com/pazi88/STM32_CAN
- SPI OLED display for immediate response
- I2C EEPROM to record milleage and parameters
- internal ADC used for precise hall reading

Compiled with the STM32duino library and the Arduino 2.0 IDE via a ST-link programmer.

Actually, the program consist of a monolithic bloc with all the stuff needed, if you want to participate and make this thing evolve, don't hesitate!
