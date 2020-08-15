#include "bme280.h"
#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef I2cHandle;

static int8_t bme280_port_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
  uint8_t dev_addr;
  dev_addr = BME280_I2C_ADDR_PRIM;

  HAL_StatusTypeDef status = HAL_OK;
  int32_t iError = 0;
  uint8_t array[40] = {0};
  uint8_t stringpos = 0;
  array[0] = reg_addr;

  while (HAL_I2C_IsDeviceReady(&I2cHandle, (uint16_t)(dev_addr<<1), 3, 100) != HAL_OK) {}

  status = HAL_I2C_Mem_Read(&I2cHandle,   // i2c handle
              (uint8_t)(dev_addr<<1), // i2c address, left aligned
              (uint8_t)reg_addr,  // register address
              I2C_MEMADD_SIZE_8BIT,     // bme280 uses 8bit register addresses
              (uint8_t*)(array),      // write returned data to this variable
              len,              // how many bytes to expect returned
              HAL_MAX_DELAY);             // timeout

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

static int8_t bme280_port_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
  uint8_t dev_addr;
  dev_addr = BME280_I2C_ADDR_PRIM;

    HAL_StatusTypeDef status = HAL_OK;
  int32_t iError = 0;

  //while (HAL_I2C_IsDeviceReady(&I2cHandle, (uint8_t)(dev_addr<<1), 3, 100) != HAL_OK) {}

    status = HAL_I2C_Mem_Write(&I2cHandle,            // i2c handle
                (uint8_t)(dev_addr<<1),   // i2c address, left aligned
                (uint8_t)reg_addr,      // register address
                I2C_MEMADD_SIZE_8BIT,     // bme280 uses 8bit register addresses
                (uint8_t*)(reg_data),   // write returned data to reg_data
                len,              // write how many bytes
                HAL_MAX_DELAY);             // timeout

  if (status != HAL_OK)
    {
        // The BME280 API calls for 0 return value as a success, and -1 returned as failure
      iError = (-1);
    }
  return (int8_t)iError;
}

static void bme280_port_delay_us(uint32_t period, void *intf_ptr)
{
  HAL_Delay(period/1000);
}

static int8_t bme280_port_stream_sensor_data_normal_mode(struct bme280_dev *dev)
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

int8_t bme280_port_sensor_init(struct bme280_dev  *dev)
{
  int8_t rslt = BME280_OK;
  uint8_t dev_addr = BME280_I2C_ADDR_PRIM;

  dev->intf_ptr = &dev_addr;
  dev->intf = BME280_I2C_INTF;
  dev->read = bme280_port_i2c_read;
  dev->write = bme280_port_i2c_write;
  dev->delay_us = bme280_port_delay_us;

  rslt = bme280_init(dev);
  if(BME280_OK != rslt)
  {
    return rslt;
  }

  rslt = bme280_port_stream_sensor_data_normal_mode(dev);

  return rslt;
}
