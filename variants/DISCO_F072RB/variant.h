/*
 *******************************************************************************
 * Copyright (c) 2017, STMicroelectronics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************
 */

#ifndef _VARIANT_ARDUINO_STM32_
#define _VARIANT_ARDUINO_STM32_

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "pins_arduino.h"

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/
extern const PinName digitalPin[];

enum {
// P1 connector
  PC13, //D0
  PC0,  //D1 - CS_I2C/SPI
  PC1,  //D2 - INT1
  PC2,  //D3 - INT2
  PC3,  //D4
  PA0,  //D5 - User button
  PA1,  //D6
  PA2,  //D7 - TS_G1_IO3
  PA3,  //D8 - TS_G1_IO4
  PA4,  //D9
  PA5,  //D10
  PA6,  //D11 - TS_G2_IO3
  PA7,  //D12 - TS_G2_IO4
  PC4,  //D13
  PC5,  //D14 - EXT_RESET
  PB0,  //D15 - TS_G3_IO2
  PB1,  //D16 - TS_G3_IO3
  PB2,  //D17
  PB10, //D18 - SCL
  PB11, //D19 - SDA
  PB12, //D20 - SPI2 SS
// P2 connector
  PB9,  //D21
  PB8,  //D22
  PB7,  //D23
  PB6,  //D24
  PB5,  //D25
  PB4,  //D26
  PB3,  //D27
  PD2,  //D28
  PC12, //D29
  PC11, //D30
  PC10, //D31
  PA15, //D32
  PA10, //D33
  PA9,  //D34
  PA8,  //D35
  PC9,  //D36 - LED green
  PC8,  //D37 - LED orange
  PC7,  //D38 - LED blue
  PC6,  //D39 - LED red
  PB15, //D40 - SPI2 MOSI
  PB14, //D41 - SPI2 MISO
  PB13, //D42 - SPI2 SCLK
  // Duplicated pins in order to be aligned with PinMap_ADC
  PA0_2,  //D43/A0 = D5 - User button
  PA1_2,  //D44/A1 = D6
  PA2_2,  //D45/A2 = D7 - TS_G1_IO3
  PA3_2,  //D46/A3 = D8 - TS_G1_IO4
  PA4_2,  //D47/A4 = D9
  PA5_2,  //D48/A5 = D10
  PA6_2,  //D49/A6 = D11 - TS_G2_IO3
  PA7_2,  //D50/A7 = D12 - TS_G2_IO4
  PB0_2,  //D51/A8 = D15 - TS_G3_IO2
  PB1_2,  //D52/A9 = D16 - TS_G3_IO3
  PC0_2,  //D53/A10 =  CS_I2C/SPI
  PC1_2,  //D54/A11 = D2 - INT1
  PC2_2,  //D55/A12 = D3 - INT2
  PC3_2,  //D56/A13 = D4
  PC4_2,  //D57/A14 = D13
  PC5_2,  //D58/A15 = D14 - EXT_RESET
  PEND
};

enum {
  A_START_AFTER = D42,
  A0,  A1,  A2,  A3,  A4,  A5,  A6,  A7,  A8,  A9,
  A10, A11, A12, A13, A14, A15,
  AEND
};

//ADC resolution is 12bits
#define ADC_RESOLUTION          12
#define DACC_RESOLUTION         12

//PWR resolution
#define PWM_RESOLUTION          8
#define PWM_FREQUENCY           1000
#define PWM_MAX_DUTY_CYCLE      255

//On-board LED pin number
#define LED_BUILTIN             36
#define LED_GREEN               LED_BUILTIN
#define LED_ORANGE              37
#define LED_BLUE                38
#define LED_RED                 39

//On-board user button
#define USER_BTN                5


//SPI definitions
#define SS                      20
#define MOSI                    40
#define MISO                    41
#define SCK                     42

//I2C Definitions
#define SDA                     19
#define SCL                     18

//Timer Definitions
//Do not use timer used by PWM pins when possible. See PinMap_PWM in PeripheralPins.c
#define TIMER_TONE              TIM14
#define TIMER_UART_EMULATED     TIM16

//Do not use basic timer: OC is required
#define TIMER_SERVO             TIM17  //TODO: advanced-control timers don't work

// UART Definitions
#define SERIAL_UART_INSTANCE    1
//#define DEBUG_UART              ((USART_TypeDef *) USART1)

// UART Emulation (uncomment if needed, required TIM1)
//#define UART_EMUL_RX            PX_n // PinName used for RX
//#define UART_EMUL_TX            PX_n // PinName used for TX

// Default pin used for 'Serial' instance (ex: ST-Link)
// Mandatory for Firmata
#define PIN_SERIAL_RX           33
#define PIN_SERIAL_TX           34

#ifdef __cplusplus
} // extern "C"
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
#define SERIAL_PORT_MONITOR   Serial
#define SERIAL_PORT_HARDWARE  Serial
#endif

#endif /* _VARIANT_ARDUINO_STM32_ */
