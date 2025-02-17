# usb-spy-current
Spy power consumption of USB

## Hardware

Board with : INA211 shunt monitor, Vbus voltage buffer (ratio 1/2), STM32L011F4Px microcontroller to measure and send measures via UART.

## Software

ADC to measure current and voltage.

UART to send data.

## Improvements
 - Drive reset line on the STM32L011
 - For more precision, use external ADC
 - SW : divide components in files
 - SW : rework transmission to not be blocking