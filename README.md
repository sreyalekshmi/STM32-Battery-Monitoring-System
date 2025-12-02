# STM32 Battery Level Indicator

This repository contains the source code and documentation for a battery
monitoring system implemented using the STM32 Nucleo-F4 microcontroller.

## Features
- ADC-based battery voltage measurement
- LED indication (High/Medium/Low)
- Dual-digit 7-segment display output
- Push-button manual sampling
- Periodic 1-second measurement

## Hardware Pins
- PA1 – ADC input (battery simulation)
- PA0 – Push button
- PA5/PA6/PA7 – LEDs
- PB0–PB7, PC0–PC7 – 7-segment display

## Folder Structure
- `Src/` contains main code
- `Docs/` contains diagrams and output images
