/*
 *******************************************************************************
 * Copyright (c) 2020, STMicroelectronics
 * All rights reserved.
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 *******************************************************************************
 */
#pragma once

/*----------------------------------------------------------------------------
 *        STM32 pins number
 *----------------------------------------------------------------------------*/
#define PA0                     PIN_A0
#define PA1                     PIN_A1
#define PA2                     PIN_A2
#define PA3                     PIN_A3
#define PA4                     PIN_A4
#define PA5                     PIN_A5
#define PA6                     PIN_A6
#define PA7                     PIN_A7
#define PA8                     PIN_A8
#define PA11                    9
#define PA12                    10
#define PA13                    PIN_A9
#define PA14                    PIN_A10
#define PA15                    13
#define PB0                     PIN_A11
#define PB1                     PIN_A12
#define PB3                     16
#define PB4                     17
#define PB5                     18
#define PB6                     19
#define PB7                     20
#define PB8                     21
#define PC6                     22
#define PC14                    23
#define PC15                    24
#define PF2                     25
#define PA9_R                   26
#define PA10_R                  27

// Alternate pins number
#define PA0_ALT1                (PA0    | ALT1)
#define PA0_ALT2                (PA0    | ALT2)
#define PA1_ALT1                (PA1    | ALT1)
#define PA1_ALT2                (PA1    | ALT2)
#define PA2_ALT1                (PA2    | ALT1)
#define PA2_ALT2                (PA2    | ALT2)
#define PA3_ALT1                (PA3    | ALT1)
#define PA3_ALT2                (PA3    | ALT2)
#define PA4_ALT1                (PA4    | ALT1)
#define PA4_ALT2                (PA4    | ALT2)
#define PA5_ALT1                (PA5    | ALT1)
#define PA5_ALT2                (PA5    | ALT2)
#define PA6_ALT1                (PA6    | ALT1)
#define PA7_ALT1                (PA7    | ALT1)
#define PA7_ALT2                (PA7    | ALT2)
#define PA7_ALT3                (PA7    | ALT3)
#define PA8_ALT1                (PA8    | ALT1)
#define PA8_ALT2                (PA8    | ALT2)
#define PA8_ALT3                (PA8    | ALT3)
#define PA8_ALT4                (PA8    | ALT4)
#define PA8_ALT5                (PA8    | ALT5)
#define PA9_R_ALT1              (PA9_R  | ALT1)
#define PA10_R_ALT1             (PA10_R | ALT1)
#define PA15_ALT1               (PA15   | ALT1)
#define PB0_ALT1                (PB0    | ALT1)
#define PB1_ALT1                (PB1    | ALT1)
#define PB1_ALT2                (PB1    | ALT2)
#define PB1_ALT3                (PB1    | ALT3)
#define PB3_ALT1                (PB3    | ALT1)
#define PB3_ALT2                (PB3    | ALT2)
#define PB5_ALT1                (PB5    | ALT1)
#define PB6_ALT1                (PB6    | ALT1)
#define PB6_ALT2                (PB6    | ALT2)
#define PB6_ALT3                (PB6    | ALT3)
#define PB6_ALT4                (PB6    | ALT4)
#define PB6_ALT5                (PB6    | ALT5)
#define PB7_ALT1                (PB7    | ALT1)
#define PB7_ALT2                (PB7    | ALT2)
#define PB7_ALT3                (PB7    | ALT3)
#define PB7_ALT4                (PB7    | ALT4)
#define PB8_ALT1                (PB8    | ALT1)
#define PC6_ALT1                (PC6    | ALT1)
#define PC14_ALT1               (PC14   | ALT1)

#define NUM_DIGITAL_PINS        28
#define NUM_REMAP_PINS          2
#define NUM_ANALOG_INPUTS       13

// On-board LED pin number
#ifndef LED_BUILTIN
  #define LED_BUILTIN           PNUM_NOT_DEFINED
#endif

// On-board user button
#ifndef USER_BTN
  #define USER_BTN              PNUM_NOT_DEFINED
#endif

// SPI definitions
#ifndef PIN_SPI_SS
  #define PIN_SPI_SS            PA4
#endif
#ifndef PIN_SPI_SS1
  #define PIN_SPI_SS1           PB0
#endif
#ifndef PIN_SPI_SS2
  #define PIN_SPI_SS2           PNUM_NOT_DEFINED
#endif
#ifndef PIN_SPI_SS3
  #define PIN_SPI_SS3           PNUM_NOT_DEFINED
#endif
#ifndef PIN_SPI_MOSI
  #define PIN_SPI_MOSI          PA7
#endif
#ifndef PIN_SPI_MISO
  #define PIN_SPI_MISO          PA6
#endif
#ifndef PIN_SPI_SCK
  #define PIN_SPI_SCK           PA5
#endif

// I2C definitions
#ifndef PIN_WIRE_SDA
  #define PIN_WIRE_SDA          PB7
#endif
#ifndef PIN_WIRE_SCL
  #define PIN_WIRE_SCL          PB6
#endif

// Timer Definitions
// Use TIM6/TIM7 when possible as servo and tone don't need GPIO output pin
#ifndef TIMER_TONE
  #define TIMER_TONE            TIM14
#endif
#ifndef TIMER_SERVO
  #define TIMER_SERVO           TIM16
#endif

// UART Definitions
#ifndef SERIAL_UART_INSTANCE
  #define SERIAL_UART_INSTANCE  1
#endif

// Default pin used for generic 'Serial' instance
// Mandatory for Firmata
#ifndef PIN_SERIAL_RX
  #define PIN_SERIAL_RX         PA1
#endif
#ifndef PIN_SERIAL_TX
  #define PIN_SERIAL_TX         PA0
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus
  // These serial port names are intended to allow libraries and architecture-neutral
  // sketches to automatically default to the correct port name for a particular type
  // of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
  // the first hardware serial port whose RX/TX pins are not dedicated to another use.
  //
  // SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
  //
  // SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
  //
  // SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
  //
  // SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
  //
  // SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
  //                            pins are NOT connected to anything by default.
  #ifndef SERIAL_PORT_MONITOR
    #define SERIAL_PORT_MONITOR   Serial
  #endif
  #ifndef SERIAL_PORT_HARDWARE
    #define SERIAL_PORT_HARDWARE  Serial
  #endif
#endif
