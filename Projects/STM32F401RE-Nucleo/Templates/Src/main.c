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
#include "bme280.h"
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
struct bme280_dev bme_dev;

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

int8_t user_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
  uint8_t dev_addr;
  dev_addr = BME280_I2C_ADDR_PRIM;

  HAL_StatusTypeDef status = HAL_OK;
  int32_t iError = 0;
  uint8_t array[40] = {0};
  uint8_t stringpos = 0;
  array[0] = reg_addr;

  while (HAL_I2C_IsDeviceReady(&I2cHandle, (uint16_t)(dev_addr<<1), 3, 100) != HAL_OK) {}

  status = HAL_I2C_Mem_Read(&I2cHandle,	  // i2c handle
              (uint8_t)(dev_addr<<1), // i2c address, left aligned
              (uint8_t)reg_addr,  // register address
              I2C_MEMADD_SIZE_8BIT,			// bme280 uses 8bit register addresses
              (uint8_t*)(array),			// write returned data to this variable
              len,							// how many bytes to expect returned
              HAL_MAX_DELAY);							// timeout

    //while (HAL_I2C_IsDeviceReady(&I2cHandle, (uint8_t)(id.dev_addr<<1), 3, 100) != HAL_OK) {}

    if (status != HAL_OK)
    {
      // The BME280 API calls for 0 return value as a success, and -1 returned as failure
      iError = (-1);
    }
  for (stringpos = 0; stringpos < len; stringpos++) {
    *(reg_data + stringpos) = array[stringpos];
  }

  return (int8_t)iError;
}

int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
  uint8_t dev_addr;
  dev_addr = BME280_I2C_ADDR_PRIM;

    HAL_StatusTypeDef status = HAL_OK;
  int32_t iError = 0;

  //while (HAL_I2C_IsDeviceReady(&I2cHandle, (uint8_t)(dev_addr<<1), 3, 100) != HAL_OK) {}

    status = HAL_I2C_Mem_Write(&I2cHandle,						// i2c handle
                (uint8_t)(dev_addr<<1),		// i2c address, left aligned
                (uint8_t)reg_addr,			// register address
                I2C_MEMADD_SIZE_8BIT,			// bme280 uses 8bit register addresses
                (uint8_t*)(reg_data),		// write returned data to reg_data
                len,							// write how many bytes
                HAL_MAX_DELAY);							// timeout

  if (status != HAL_OK)
    {
        // The BME280 API calls for 0 return value as a success, and -1 returned as failure
      iError = (-1);
    }
  return (int8_t)iError;
}

void user_delay_us(uint32_t period, void *intf_ptr)
{
	HAL_Delay(period/1000);
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

int8_t bme280_sensor_init(struct bme280_dev  *dev)
{
  int8_t rslt = BME280_OK;
  uint8_t dev_addr = BME280_I2C_ADDR_PRIM;
  uint8_t sensor_mode;

  dev->intf_ptr = &dev_addr;
  dev->intf = BME280_I2C_INTF;
  dev->read = user_i2c_read;
  dev->write = user_i2c_write;
  dev->delay_us = user_delay_us;

  rslt = bme280_init(dev);
  bme280_get_sensor_mode(&sensor_mode, dev);
  return rslt;
}

int8_t stream_sensor_data_normal_mode(struct bme280_dev *dev)
{
  int8_t rslt;
  uint8_t settings_sel;

  /* Recommended mode of operation: Indoor navigation */
  dev->settings.osr_h = BME280_OVERSAMPLING_1X;
  dev->settings.osr_p = BME280_OVERSAMPLING_16X;
  dev->settings.osr_t = BME280_OVERSAMPLING_2X;
  dev->settings.filter = BME280_FILTER_COEFF_16;
  dev->settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

  settings_sel = BME280_OSR_PRESS_SEL;
  settings_sel |= BME280_OSR_TEMP_SEL;
  settings_sel |= BME280_OSR_HUM_SEL;
  settings_sel |= BME280_STANDBY_SEL;
  settings_sel |= BME280_FILTER_SEL;
  rslt = bme280_set_sensor_settings(settings_sel, dev);
  rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, dev);

  return rslt;
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

void ssd1306_print_measurements(int32_t temperature, uint32_t humidity, uint32_t pressure)
{
  char buffer[20];

  ssd1306_Fill(White);
  ssd1306_SetCursor(2, 0);
  sprintf(buffer, "%d.%d C\n\r",temperature / 100, (temperature % 100) / 10);
  ssd1306_WriteString(buffer, Font_11x18, Black);
  ssd1306_DrawCircle(54,3,2,Black);
  ssd1306_SetCursor(2,18);
  sprintf(buffer, "%d.%d %%\n\r", humidity /1024, (humidity % 1024) / 102);
  ssd1306_WriteString(buffer, Font_11x18, Black);
  ssd1306_SetCursor(2,36);
  sprintf(buffer, "%d.%1.d hPa\n\r",pressure /100, (pressure % 100) / 10);
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
  bme280_sensor_init(&bme_dev);

  struct bme280_data bme280_d;

  ssd1306_Init();
  ssd1306_WriteCommand(0xAE); //display off

  stream_sensor_data_normal_mode(&bme_dev);

  while(1)
  {
	bme280_get_sensor_data(BME280_ALL, &bme280_d, &bme_dev);
    if(displayActive)
    {
      sprintf(buffer, "T:%d, H:%d P:%d\n\r",bme280_d.temperature, bme280_d.humidity, bme280_d.pressure);
      HAL_UART_Transmit(&UartHandle, (uint8_t*)buffer, strlen(buffer), 10);
      ssd1306_print_measurements(bme280_d.temperature, bme280_d.humidity, bme280_d.pressure);
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
