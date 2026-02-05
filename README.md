# VESC-CAN-OLED-display
Simple display for VESC controllers, allowing for full control via CAN bus

The program is built arround the STM32F103CBT6 that uses many of it's functions:

- native CAN implementation thanks to the STM32_CAN library from pazi88: https://github.com/pazi88/STM32_CAN
- SPI OLED display for immediate response thanks the the U8g2_Arduino from olikraus: https://github.com/olikraus/U8g2_Arduino
- I2C EEPROM to record milleage and parameters thansk to the arduino-library-at24cxxx from stefangs: https://github.com/stefangs/arduino-library-at24cxxx
- internal ADC used for precise hall reading with SimpleKalmanFilter from denyssene: https://github.com/denyssene/SimpleKalmanFilter

Compiled with the lastest STM32duino library and the Arduino 2.0 IDE via a ST-link programmer.

Actually, the program consist of a monolithic bloc with all the stuff needed, if you want to participate and make this thing evolve, don't hesitate!

PCB: https://oshwlab.com/marouane0900/trottinette-ecran-can-v1-2r-tja-0603

If you want, I actually have few board wihout display for sale that you can find in the next link below.

Soldered main board: [https://www.ebay.fr/itm/257270297207](https://www.ebay.fr/itm/257270612879)

OLED display: [https://fr.aliexpress.com/item/33044134713.html](https://aliexpress.com/item/33044134713.html)

Cults3D repository for enclosure: [https://cults3d.com/en/3d-model/gadget/vesc-can-oled-display-enclosure](https://cults3d.com/en/3d-model/gadget/vesc-can-oled-display-enclosure)

