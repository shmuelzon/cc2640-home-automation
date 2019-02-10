/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/** ============================================================================
 *  @file       SensorBme280.c
 *
 *  @brief      Driver for the Bosch BME280 Environmental Sensor.
 *  ============================================================================
 */

/* -----------------------------------------------------------------------------
*  Includes
* ------------------------------------------------------------------------------
*/
#include "board.h"
#include "SensorBme280.h"
#include "SensorUtil.h"
#include "SensorI2C.h"
#include "bme280.h"

/* -----------------------------------------------------------------------------
*  Local Variables
* ------------------------------------------------------------------------------
*/

static int8_t _i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
static int8_t _i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
static void _delay_ms(uint32_t period);

static struct bme280_dev dev = {
  .dev_id = Board_BME280_ADDR,
  .intf = BME280_I2C_INTF,
  .read = _i2c_read,
  .write = _i2c_write,
  .delay_ms = _delay_ms,
  .settings = {
    .osr_p = BME280_OVERSAMPLING_4X,
    .osr_t = BME280_OVERSAMPLING_4X,
    .osr_h = BME280_OVERSAMPLING_4X,
  },
};

/* -----------------------------------------------------------------------------
*  Private functions
* ------------------------------------------------------------------------------
*/

static int8_t _i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  if (!SensorI2C_select(SENSOR_I2C_0, dev_id))
    return -1;

  SensorI2C_readReg(reg_addr, data, len);

  SensorI2C_deselect();

  return 0;
}

static int8_t _i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  if (!SensorI2C_select(SENSOR_I2C_0, dev_id))
    return false;

  SensorI2C_writeReg(reg_addr, data, len);

  SensorI2C_deselect();

  return 0;
}

static void _delay_ms(uint32_t period)
{
  Task_sleep(period * 1000 / Clock_tickPeriod);
}

/* -----------------------------------------------------------------------------
*  Public functions
* ------------------------------------------------------------------------------
*/

/*******************************************************************************
 * @fn          SensorBme280_init
 *
 * @brief       Initialize the sensor
 *
 * @return      true if success
 */
bool SensorBme280_init(void)
{
  if (bme280_init(&dev) != BME280_OK)
	  return false;

  if (bme280_set_sensor_settings(BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL, &dev) != BME280_OK)
    return false;

  return true;
}


/*******************************************************************************
 * @fn          SensorBme280_enable
 *
 * @brief       Enable/disable measurements
 *
 * @param       enable - flag to turn the sensor on/off
 *
 * @return      none
 */
void SensorBme280_enable(bool enable)
{
  if (enable)
    bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
  else
    bme280_set_sensor_mode(BME280_SLEEP_MODE, &dev);
}


/*******************************************************************************
 * @fn          SensorBme280_read
 *
 * @brief       Read temperature and pressure data
 *
 * @param       temp - pointer for storing the temperature value
 * @param       press - pointer for storing the pressure value
 * @param       hum - pointer for storing the humidity value
 *
 * @return      TRUE if valid data
 */
bool SensorBme280_read(int32_t *temp, uint32_t *press, uint32_t *hum)
{
  struct bme280_data comp_data;

  if (bme280_get_sensor_data(BME280_ALL, &comp_data, &dev) != BME280_OK)
    return false;

  *temp = comp_data.temperature;
  *press = comp_data.pressure;
  *hum = comp_data.humidity;

  return true;
}

/*******************************************************************************
 * @fn          SensorBme280_measurementTime
 *
 * @brief       Return the length of measurement time
 *
 * @return      Number of milliseconds for measurement
 *
 ******************************************************************************/
uint16_t SensorBme280_measurementTime(void)
{
  /* Calculated as:
   * 1.25 + (2.3 * (1 << (T_Oversampling - 1))) +
   * (2.3 * (1 << (P_Oversampling - 1)) + 0.575) +
   * (2.3 * (1 << (H_Oversampling - 1)) + 0.575)
   */
  return 58;
}
