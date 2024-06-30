# Square Waveform Generator on Atmel AT91SAM7

**by**: Min Khant Soe Oke, Kaung Sithu

## Overview

This project involves developing a square waveform generator using the Atmel AT91SAM7 microcontroller, the PWM controller, and the ADC for frequency adjustment. The generator output signal drives a speaker on the SAM7-EX256 evaluation board. This project also covers configuring and handling the periodic interval timer (PIT) for periodic program loop execution.

## Dependencies

**To run this project, you need to have the following dependencies installed:**

* Atmel AT91SAM7X256 microcontroller
* Olimex SAM7-EX256 evaluation board
* IAR Embedded Workbench
* Basic knowledge of C programming

## Features

**Analog-to-Digital Converter (ADC) Handling**
- Adjust the frequency of the generator using a potentiometer connected to the ADC.

**PWM Controller (PWMC) Handling**
- Configure and manage the PWM controller to generate square waveforms with adjustable frequency and duty cycle.

**Periodic Interval Timer (PIT) Handling**
- Use the PIT to trigger periodic execution of the program loop, ensuring consistent timing for waveform generation.

**Interrupt Handling**
- Configure and manage interrupts using the Advanced Interrupt Controller (AIC).

## How to Use

1. **Start the Main Program**:
   - Open a terminal and run `./main`.

2. **ADC Configuration**:
   - The ADC converts the signal from the potentiometer to adjust the frequency of the generator.

3. **PWMC Configuration**:
   - Configure PWMC channel 0 to generate square waveforms with the desired frequency and duty cycle.

4. **PIT and AIC Configuration**:
   - Use the PIT to trigger periodic execution of the main loop and configure interrupts using the AIC.

5. **Display Information**:
   - Display the generator's current state, frequency, and duty cycle on the LCD screen.

## Authors

- Min Khant Soe Oke
- Kaung Sithu
