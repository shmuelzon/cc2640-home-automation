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
 *  @file       SensorOpt3001.c
 *
 *  @brief      Driver for the Texas Instruments OP3001 Optical Sensor.
 *  ============================================================================
 */

/* -----------------------------------------------------------------------------
*  Includes
* ------------------------------------------------------------------------------
*/
#include "board.h"
#include "SensorOpt3001.h"
#include "SensorUtil.h"
#include "SensorI2C.h"
#include "math.h"

/* -----------------------------------------------------------------------------
*  Constants and macros
* ------------------------------------------------------------------------------
*/

/* Register addresses */
#define REG_RESULT                      0x00
#define REG_CONFIGURATION               0x01
#define REG_LOW_LIMIT                   0x02
#define REG_HIGH_LIMIT                  0x03

#define REG_MANUFACTURER_ID             0x7E
#define REG_DEVICE_ID                   0x7F

/* Register values (little endian) */
#define MANUFACTURER_ID                 0x4954  // 0x5449
#define DEVICE_ID                       0x0130  // 0x3001
#define CONFIG_ENABLE                   0x10C2  // 0xC210   - 100 ms, single-shot
#define CONFIG_DISABLE                  0x10C0  // 0xC010   - 100 ms, shut-down

/* Bit values */
#define DATA_RDY_BIT                    0x8000  // Config: 0x0080 = Data ready

/* Register length */
#define REGISTER_LENGTH                 2

/* Sensor data size */
#define DATA_LENGTH                     2

// Sensor selection/de-selection
#define SENSOR_SELECT()     SensorI2C_select(SENSOR_I2C_0, Board_OPT3001_ADDR)
#define SENSOR_DESELECT()   SensorI2C_deselect()

/* -----------------------------------------------------------------------------
*  Public functions
* ------------------------------------------------------------------------------
*/

/*******************************************************************************
 * @fn          SensorOpt3001_init
 *
 * @brief       Initialize the temperature sensor driver
 *
 * @return      none
 ******************************************************************************/
bool SensorOpt3001_init(void)
{
  SensorOpt3001_enable(false);

  return true;
}

/*******************************************************************************
 * @fn          SensorOpt3001_enable
 *
 * @brief       Turn the sensor on/off
 *
 * @return      none
 ******************************************************************************/
void SensorOpt3001_enable(bool enable)
{
  uint16_t val;

  if (!SENSOR_SELECT())
    return;

  if (enable)
    val = CONFIG_ENABLE;
  else
    val = CONFIG_DISABLE;

  SensorI2C_writeReg(REG_CONFIGURATION, (uint8_t *)&val, REGISTER_LENGTH);

  SENSOR_DESELECT();
}

/*******************************************************************************
 * @fn          SensorOpt3001_read
 *
 * @brief       Read the result register
 *
 * @param       lux - pointer for storing the lux value
 *
 * @return      true if valid data
 ******************************************************************************/
bool SensorOpt3001_read(float *lux)
{
  uint16_t val, e, m;

  if (!SENSOR_SELECT())
    return false;

  ST_ASSERT(SensorI2C_readReg(REG_CONFIGURATION, (uint8_t *)&val, REGISTER_LENGTH) == TRUE);
  ST_ASSERT((val & DATA_RDY_BIT) == DATA_RDY_BIT);
  ST_ASSERT(SensorI2C_readReg(REG_RESULT, (uint8_t*)&val, DATA_LENGTH) == TRUE);

  val = (val << 8) | ((val >> 8) & 0xFF); // Swap bytes
  m = val & 0x0FFF;
  e = (val & 0xF000) >> 12;
  *lux = m * (0.01 * exp2(e));

  SENSOR_DESELECT();

  return true;
}

/*******************************************************************************
 * @fn          SensorOpt3001_measurementTime
 *
 * @brief       Return the length of measurement time
 *
 * @return      Number of milliseconds for measurement
 *
 ******************************************************************************/
uint16_t SensorOpt3001_measurementTime(void)
{
  return 150;
}
