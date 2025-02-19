# usb-spy-current
Spy power consumption of USB

## Hardware

Board with : INA211 shunt monitor, Vbus voltage buffer (ratio 1/2), STM32L011F4Px microcontroller to measure and send measures via UART.

## Software

ADC to measure current and voltage.

UART to send data.

## Python script

Read incoming serial data and display it on graph.

## TODO : Variant
 - Do a variant to change from Man in the Middle attack to power supply
    - Only one USB port / terminal block
    - Voltage range choice : 5V - 3.3V - 1.8V
    - For more precision, use external ADC
      - No uC on board, the User will have to connect to the ADC directly
    - Coaxial around the shunt
    - Chose shunt according to application : choice possible on board 1 - 5 - 10 - 100 Ohm for example