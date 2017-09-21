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

#include "variant.h"

#ifdef __cplusplus
extern "C" {
#endif

// Pin number following UM1690 figure 3 and table 6
const PinName digitalPin[] = {
// P1 connector
  PC_13, //D0
  PC_0,  //D1 - CS_I2C/SPI
  PC_1,  //D2 - INT1
  PC_2,  //D3 - INT2
  PC_3,  //D4
  PA_0,  //D5 - User button
  PA_1,  //D6
  PA_2,  //D7 - TS_G1_IO3
  PA_3,  //D8 - TS_G1_IO4
  PA_4,  //D9
  PA_5,  //D10
  PA_6,  //D11 - TS_G2_IO3
  PA_7,  //D12 - TS_G2_IO4
  PC_4,  //D13
  PC_5,  //D14 - EXT_RESET
  PB_0,  //D15 - TS_G3_IO2
  PB_1,  //D16 - TS_G3_IO3
  PB_2,  //D17
  PB_10, //D18 - SCL
  PB_11, //D19 - SDA
  PB_12, //D20 - SPI2 SS
// P2 connector
  PB_9,  //D21
  PB_8,  //D22
  PB_7,  //D23
  PB_6,  //D24
  PB_5,  //D25
  PB_4,  //D26
  PB_3,  //D27
  PD_2,  //D28
  PC_12, //D29
  PC_11, //D30
  PC_10, //D31
  PA_15, //D32
  PA_10, //D33
  PA_9,  //D34
  PA_8,  //D35
  PC_9,  //D36 - LED green
  PC_8,  //D37 - LED orange
  PC_7,  //D38 - LED blue
  PC_6,  //D39 - LED red
  PB_15, //D40 - SPI2 MOSI
  PB_14, //D41 - SPI2 MISO
  PB_13, //D42 - SPI2 SCLK
  // Duplicated pins in order to be aligned with PinMap_ADC
  PA_0, //D43/A0 = D5 - User button
  PA_1, //D44/A1 = D6
  PA_2, //D45/A2 = D7 - TS_G1_IO3
  PA_3, //D46/A3 = D8 - TS_G1_IO4
  PA_4, //D47/A4 = D9
  PA_5, //D48/A5 = D10
  PA_6, //D49/A6 = D11 - TS_G2_IO3
  PA_7, //D50/A7 = D12 - TS_G2_IO4
  PB_0, //D51/A8 = D15 - TS_G3_IO2
  PB_1, //D52/A9 = D16 - TS_G3_IO3
  PC_0, //D53/A10 =  CS_I2C/SPI
  PC_1, //D54/A11 = D2 - INT1
  PC_2, //D55/A12 = D3 - INT2
  PC_3, //D56/A13 = D4
  PC_4, //D57/A14 = D13
  PC_5  //D58/A15 = D14 - EXT_RESET
};

#ifdef __cplusplus
}
#endif

/*
 * UART objects
 */
// Replace PX_n_Rx and PX_n_Tx by UART_RX and UART_TX pin names
HardwareSerial  Serial(PA_10, PA_9); // Connected to ST-Link

void serialEvent() __attribute__((weak));
void serialEvent() { }

// Define as many Serial instance as desired
// Replace 'SerialX' by the desired name
//#ifdef ENABLE_SERIAL1
//HardwareSerial  Serial1(PA_10, PA_9);
//
//void serialEvent1() __attribute__((weak));
//void serialEvent1() { }
//#endif

void serialEventRun(void)
{
  if (Serial.available()) serialEvent();
//#ifdef ENABLE_SERIAL1
//  if (Serial1.available()) serialEvent1();
//#endif
}

// ----------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  System Clock Configuration
  * @param  None
  * @retval None
  */
WEAK void SystemClock_Config(void)
{

	  RCC_OscInitTypeDef RCC_OscInitStruct;
	  RCC_ClkInitTypeDef RCC_ClkInitStruct;
	  RCC_PeriphCLKInitTypeDef PeriphClkInit;

	    /**Initializes the CPU, AHB and APB busses clocks
	    */
	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
	  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	  {
	    while(1);
	  }

	    /**Initializes the CPU, AHB and APB busses clocks
	    */
	  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                              |RCC_CLOCKTYPE_PCLK1;
	  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
	  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	  {
	    while(1);
	  }

	  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1;
	  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
	  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

	  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	  {
	    while(1);
	  }

	    /**Configure the Systick interrupt time
	    */
	  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	    /**Configure the Systick
	    */
	  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	  /* SysTick_IRQn interrupt configuration */
	  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

#ifdef __cplusplus
}
#endif
