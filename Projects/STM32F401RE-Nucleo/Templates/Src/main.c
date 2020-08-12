/**
  ******************************************************************************
  * @file    Templates/Src/main.c 
  * @author  MCD Application Team
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
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
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "ssd1306.h"
#include "string.h"
#include <stdio.h>

/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup Templates
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define I2C_MASTER_ADDRESS        0xFF
#define I2C_TIMING      0x40912732
#define I2C_SLAVE_ADDRESS 0x30
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef UartHandle;
I2C_HandleTypeDef I2cHandle;
TIM_HandleTypeDef TimHandle;
volatile uint8_t displayActive = 0;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
static void EXTILine8_Config(void);

/* Private functions ---------------------------------------------------------*/

void UartInit(void)
{
  UartHandle.Instance        = USART2;
  UartHandle.Init.BaudRate   = 115200;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits   = UART_STOPBITS_1;
  UartHandle.Init.Parity     = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
  UartHandle.Init.Mode       = UART_MODE_TX_RX;

  HAL_UART_Init(&UartHandle);
}

void I2cInit(void)
{
  /*##-1- Configure the I2C peripheral ######################################*/
  I2cHandle.Instance             = I2Cx;
  I2cHandle.Init.ClockSpeed      = 400000;
  I2cHandle.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  I2cHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  I2cHandle.Init.DutyCycle       = I2C_DUTYCYCLE_16_9;
  I2cHandle.Init.OwnAddress1     = I2C_MASTER_ADDRESS;
  I2cHandle.Init.OwnAddress2     = 0xFE;
  I2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  I2cHandle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;

  if(HAL_I2C_Init(&I2cHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

void TimerInit(void)
{
  uint32_t uwPrescalerValue;
  /* Compute the prescaler value to have TIMx counter clock equal to 10000 Hz */
  uwPrescalerValue = (uint32_t)(SystemCoreClock / 10000) - 1;

  /* Set TIMx instance */
  TimHandle.Instance = TIMx;

  /* Initialize TIMx peripheral as follows:
       + Period = 50000 - 1
       + Prescaler = ((SystemCoreClock / 2)/10000) - 1
       + ClockDivision = 0
       + Counter direction = Up
  */
  TimHandle.Init.Period            = 50000 - 1;
  TimHandle.Init.Prescaler         = uwPrescalerValue;
  TimHandle.Init.ClockDivision     = 0;
  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimHandle.Init.RepetitionCounter = 0;
  TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

#define POSITIVE_TEMPERATURE_RANGE 0x7FF

#define MCP9808_TEMPERATURE_AMBIENT 0x05
#define MCP9808_MANUFACTURER_ID 0x06

// Example of temperature conversion. Potentially some errors there (i.e negative temperature)
// https://www.digikey.pl/en/maker/projects/getting-started-with-stm32-i2c-example/ba8c2bfef2024654b5dd10012425fa23

void mcp9808_read_temperature(char* result_buffer)
{
  uint16_t temperature;
  float f_temperature;
  HAL_I2C_Mem_Read(&I2cHandle, I2C_SLAVE_ADDRESS, MCP9808_TEMPERATURE_AMBIENT, 1, (uint8_t*)&temperature, sizeof(temperature), HAL_MAX_DELAY);
  /* swap beacuse of endianess */
  temperature = __REV16(temperature);
  /* ignore alert pin state bits */
  temperature = temperature & 0x1fff;
  if ( temperature > POSITIVE_TEMPERATURE_RANGE)
  {
    temperature = ~temperature;
    temperature+=1;
    temperature = temperature & 0x7ff;
    f_temperature = temperature * 0.0625;
    f_temperature *= 100;
    sprintf(result_buffer, "-%d.%d C\n\r", (int)f_temperature / 100, ((int) f_temperature % 100)/10);
  }
  else
  {
    temperature = temperature & 0x7ff;
    f_temperature = temperature * 0.0625;
    f_temperature *= 100;
    sprintf(result_buffer, "%d.%d C\n\r", (int)f_temperature / 100, ((int) f_temperature % 100)/10);
  }
}

uint16_t mcp9808_read_manufacturer_id(char* buffer)
{
  uint16_t manufacturer_id=0xabcd;

  HAL_I2C_Mem_Read(&I2cHandle, I2C_SLAVE_ADDRESS, MCP9808_MANUFACTURER_ID, 1, (uint8_t*)&manufacturer_id, sizeof(manufacturer_id), HAL_MAX_DELAY);

  manufacturer_id = __REV16(manufacturer_id);
  sprintf(buffer, "manufacturer id: 0x%02x\r\n", manufacturer_id);
  return manufacturer_id;
}

void ssd1306_print_temperature(char* buffer)
{
  ssd1306_Fill(White);
  ssd1306_SetCursor(2, 0);
  ssd1306_WriteString("Temperature", Font_11x18, Black);
  ssd1306_SetCursor(2,18);
  ssd1306_WriteString(buffer, Font_11x18, Black);
  ssd1306_UpdateScreen();
}

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  char buffer[40];
  uint16_t result;

  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, Flash preread and Buffer caches
       - Systick timer is configured by default as source of time base, but user
             can eventually implement his proper time base source (a general purpose
             timer for example or other time source), keeping in mind that Time base
             duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
             handled in milliseconds basis.
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure the System clock to have a frequency of 216 MHz */
  SystemClock_Config();

  /* -1- Enable GPIOA Clock (to be able to program the configuration registers) */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  EXTILine8_Config();
  UartInit();
  I2cInit();
  TimerInit();


  result = mcp9808_read_manufacturer_id(buffer);
  (void)result;
  assert_param(result == 0x54);

  HAL_UART_Transmit(&UartHandle, (uint8_t*)buffer, strlen(buffer), 10);

  ssd1306_Init();
  ssd1306_WriteCommand(0xAE); //display off

  while(1)
  {
    /*##-5- Get the converted value of regular channel  ########################*/
    mcp9808_read_temperature(buffer);

    if(displayActive)
    {
		HAL_UART_Transmit(&UartHandle, (uint8_t*)buffer, strlen(buffer), 10);
		ssd1306_print_temperature(buffer);
    }
    HAL_Delay(1000);
  }
}


/**
  * @brief  Configures EXTI Line8 (connected to PA8 pin) in interrupt mode
  * @param  None
  * @retval None
  */
static void EXTILine8_Config(void)
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOA clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pin = GPIO_PIN_6;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6, GPIO_PIN_SET);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	displayActive = 0;
	ssd1306_WriteCommand(0xAE); //display off
	HAL_TIM_Base_Stop_IT(&TimHandle);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_6)
  {
	  displayActive = 1;
	  ssd1306_WriteCommand(0xAF); //display on
	  HAL_TIM_Base_Start_IT(&TimHandle);
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 84000000
  *            HCLK(Hz)                       = 84000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSI Frequency(Hz)              = 16000000
  *            PLL_M                          = 16
  *            PLL_N                          = 336
  *            PLL_P                          = 4
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale2 mode
  *            Flash Latency(WS)              = 2 
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /* Enable HSI Oscillator and activate PLL with HSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 0x10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
 
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  I2C error callbacks.
  * @param  I2cHandle: I2C handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
 void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2cHandle)
{
  HAL_Delay(100);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1)
  {
    HAL_Delay(100);
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  char buffer[80];
  sprintf(buffer, "\r\nassert_failed(). file: %s, line: %ld\r\n", (char *) file, line );
  HAL_UART_Transmit(&UartHandle, (uint8_t*)buffer, strlen(buffer), 10);
  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
