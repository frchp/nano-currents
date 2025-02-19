# NanoCurrents
Spy power consumption.

## Repository

This repository combines hardware and software files for my power consumption projects.

## Hardware

Hardware files for :
  - [NanoCurrents](hw/NanoCurrents_v2.pdf), a way to amplify easily with a small board currents from the nA range, using a INA211 shunt monitor and multiple shunt footprint to use.
  - [NanoCurrents4USB](hw/NanoCurrents4USB.pdf), a way to monitor power consumption on an USB bus, with an integrated STM32L011F4Px microcontroller to measure and send measures via UART. Based on an INA211 shunt monitor, and a basic voltage buffer (ratio 1/2).
  - [NanoCurrentsSupply](hw/NanoCurrentsSupply.pdf), a way to supply three differents voltage to a target and monitor the power consumption (INA240) with the integrated ADC (ADS1015).

## Software

Software for NanoCurrents4USB :
  - Firmware to run on STM32L011F4Px : acquire data and send it via UART.
  - Python script to get UART from COM port on computer and display data in real time.

## Doc

Interesting documentation for power consumption and side power analysis:
 - [Texas Instruments documentation on low side current measurement](doc/AN_texas_Low-Side%20Current%20Sense%20Circuit%20Integration.pdf)
 - [Paper on side power analysis methodology](doc/introduction_to_differential_power_analysis.pdf)
 - [STM32 Reference manual](doc/rm0377-ultralowpower-stm32l0x1-advanced-armbased-32bit-mcus-stmicroelectronics.pdf) and [STM32 datasheet](doc/stm32l011f4.pdf)
 - [Methodology to chose resistor based on DUT](doc/testing_methodology_for_side_channel_resistance_validation.pdf)