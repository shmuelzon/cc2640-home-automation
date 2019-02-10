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
 *  @file       board.h
 *
 *  @brief      Board header file. Based on CC2650DK_4XS.h
 *              The project options should point to this file if this is the
 *              CC2650EM you are developing code for.
 *
 *  The CC2650 header file should be included in an application as follows:
 *  @code
 *  #include <Board.h>
 *  @endcode
 *
 *  ============================================================================
 */
#ifndef __BOARD_H__
#define __BOARD_H__

#ifdef __cplusplus
extern "C" {
#endif

/** ============================================================================
 *  Symbol by generic Board.c to include the correct kit specific Board.c
 *  Needed by ble_user_config.[ch].
 *  ==========================================================================*/
#if defined(CC2650_LAUNCHXL) || defined(CC2650DK_7ID) || defined(CC2650RC) || defined(CC2650STK)
#define CC2650EM_7ID
#elif defined(BOOSTXL_CC2650MA)
#define CC2650M5A
#elif defined(CC2650DK_5XD)
#define CC2650EM_5XD
#elif defined(CC2650DK_4XS)
#define CC2650EM_4XS
#else
#error Unkown board definition
#endif

/** ============================================================================
 *  Includes
 *  ==========================================================================*/
#include <ti/drivers/PIN.h>
#include <driverlib/ioc.h>

/** ============================================================================
 *  Externs
 *  ==========================================================================*/
extern const PIN_Config BoardGpioInitTable[];

/** ============================================================================
 *  Defines
 *  ==========================================================================*/

/* Mapping of pins to board signals using general board aliases
 *      <board signal alias>                <pin mapping>
 */
/* Leds */
#define Board_LED_ON                        1 /* LEDs are active high */
#define Board_LED_OFF                       0
#define Board_LED1                          PIN_UNASSIGNED
#define Board_LED2                          PIN_UNASSIGNED
#define Board_LED3                          PIN_UNASSIGNED
#define Board_LED4                          PIN_UNASSIGNED
/* Button Board */
#define Board_KEY_SELECT                    PIN_UNASSIGNED
#define Board_KEY_UP                        PIN_UNASSIGNED
#define Board_KEY_DOWN                      PIN_UNASSIGNED
#define Board_KEY_LEFT                      PIN_UNASSIGNED
#define Board_KEY_RIGHT                     PIN_UNASSIGNED
/* LCD  Board */
#define Board_3V3_EN                        PIN_UNASSIGNED
#define Board_LCD_MODE                      PIN_UNASSIGNED
#define Board_LCD_RST                       PIN_UNASSIGNED
#define Board_LCD_CSN                       PIN_UNASSIGNED
/* UART Board */
#define Board_UART_RX                       PIN_UNASSIGNED
#define Board_UART_TX                       PIN_UNASSIGNED
#define Board_UART_CTS                      PIN_UNASSIGNED
#define Board_UART_RTS                      PIN_UNASSIGNED
/* I2C */
#define Board_I2C0_SDA0                     PIN_UNASSIGNED
#define Board_I2C0_SCL0                     PIN_UNASSIGNED
#define Board_I2C0_SDA1                     PIN_UNASSIGNED
#define Board_I2C0_SCL1                     PIN_UNASSIGNED
/* SPI Board */
#define Board_SPI0_MISO                     PIN_UNASSIGNED
#define Board_SPI0_MOSI                     PIN_UNASSIGNED
#define Board_SPI0_CLK                      PIN_UNASSIGNED
#define Board_SPI0_CSN                      PIN_UNASSIGNED
/* PWM outputs */
#define Board_PWMPIN0                       PIN_UNASSIGNED
#define Board_PWMPIN1                       PIN_UNASSIGNED
#define Board_PWMPIN2                       PIN_UNASSIGNED
#define Board_PWMPIN3                       PIN_UNASSIGNED
#define Board_PWMPIN4                       PIN_UNASSIGNED
#define Board_PWMPIN5                       PIN_UNASSIGNED
#define Board_PWMPIN6                       PIN_UNASSIGNED
#define Board_PWMPIN7                       PIN_UNASSIGNED
/* Outputs */
#define Board_RELAY_SET                     IOID_7
#define Board_RELAY_RESET                   IOID_8
#define Board_WS2812_SPI                    Board_SPI0
#define Board_WS2812_NUM_LEDS               0
#define Board_RGB_NUM_LEDS                  (Board_WS2812_NUM_LEDS)
/* Sensors */
#define Board_SWITCH                        IOID_5
#define Board_BUTTON                        PIN_UNASSIGNED
#define Board_CONTACT                       PIN_UNASSIGNED
#define Board_MOTION                        PIN_UNASSIGNED

/* Battery levels (mV) */
#define Board_BATT_MAX                      3000
#define Board_BATT_MIN                      2100

/* Board specific I2C addresses */
#define Board_OPT3001_ADDR                  0
#define Board_BME280_ADDR                   0

/** ============================================================================
 *  Instance identifiers
 *  ==========================================================================*/
/* Generic I2C instance identifiers */
#define Board_I2C                   CC26XX_I2C0
/* Generic SPI instance identifiers */
#define Board_SPI0                  CC26XX_SPI0
/* Generic UART instance identifiers */
#define Board_UART                  CC26XX_UART0
/* Generic Crypto instance identifiers */
#define Board_CRYPTO                CC26XX_CRYPTO0
/* Generic GPTimer instance identifiers */
#define Board_GPTIMER0A             CC26XX_GPTIMER0A
#define Board_GPTIMER0B             CC26XX_GPTIMER0B
#define Board_GPTIMER1A             CC26XX_GPTIMER1A
#define Board_GPTIMER1B             CC26XX_GPTIMER1B
#define Board_GPTIMER2A             CC26XX_GPTIMER2A
#define Board_GPTIMER2B             CC26XX_GPTIMER2B
#define Board_GPTIMER3A             CC26XX_GPTIMER3A
#define Board_GPTIMER3B             CC26XX_GPTIMER3B
/* Generic PWM instance identifiers */
#define Board_PWM0                  CC26XX_PWM0
#define Board_PWM1                  CC26XX_PWM1
#define Board_PWM2                  CC26XX_PWM2
#define Board_PWM3                  CC26XX_PWM3
#define Board_PWM4                  CC26XX_PWM4
#define Board_PWM5                  CC26XX_PWM5
#define Board_PWM6                  CC26XX_PWM6
#define Board_PWM7                  CC26XX_PWM7
/* Generic TRNG instance identifier */
#define Board_TRNG                  CC26XX_TRNG0
/* Generic ADC instance identifier */
#define Board_ADC0                  CC26XX_ADC0
#define Board_ADC1                  CC26XX_ADC1
#define Board_ADC2                  CC26XX_ADC2
#define Board_ADC3                  CC26XX_ADC3
#define Board_ADC4                  CC26XX_ADC4
#ifndef CC2650DK_4XS
#define Board_ADC5                  CC26XX_ADC5
#define Board_ADC6                  CC26XX_ADC6
#define Board_ADC7                  CC26XX_ADC7
#endif
#define Board_ADCDCOUPL             CC26XX_ADCDCOUPL
#define Board_ADCVSS                CC26XX_ADCVSS
#define Board_ADCVDDS               CC26XX_ADCVDDS

/** ============================================================================
 *  Number of peripherals and their names
 *  ==========================================================================*/

/*!
 *  @def    CC2650STK_I2CName
 *  @brief  Enum of I2C names on the board
 */
typedef enum CC26XX_I2CName {
    CC26XX_I2C0 = 0,
    CC26XX_I2CCOUNT
} CC26XX_I2CName;

/*!
 *  @def    CC26XX_CryptoName
 *  @brief  Enum of Crypto names on the board
 */
typedef enum CC26XX_CryptoName {
    CC26XX_CRYPTO0 = 0,
    CC26XX_CRYPTOCOUNT
} CC26XX_CryptoName;

/*!
 *  @def    CC26XX_SPIName
 *  @brief  Enum of SPI names on the board
 */
typedef enum CC26XX_SPIName {
    CC26XX_SPI0 = 0,
    CC26XX_SPICOUNT
} CC26XX_SPIName;

/*!
 *  @def    CC26XX_TRNGName
 *  @brief  Enum of TRNG names on the board
 */
typedef enum CC26XX_TRNGName {
    CC26XX_TRNG0 = 0,
    CC26XX_TRNGCOUNT
} CC26XX_TRNGName;

/*!
 *  @def    CC26XX_UARTName
 *  @brief  Enum of UARTs on the CC2650 dev board
 */
typedef enum CC26XX_UARTName {
    CC26XX_UART0 = 0,
    CC26XX_UARTCOUNT
} CC26XX_UARTName;

/*!
 *  @def    CC26XX_UdmaName
 *  @brief  Enum of DMA buffers
 */
typedef enum CC26XX_UdmaName {
    CC26XX_UDMA0 = 0,
    CC26XX_UDMACOUNT
} CC26XX_UdmaName;

/*!
 *  @def    CC26XX_GPTimerName
 *  @brief  Enum of GPTimer parts
 */
typedef enum CC26XX_GPTimerName
{
    CC26XX_GPTIMER0A = 0,
    CC26XX_GPTIMER0B,
    CC26XX_GPTIMER1A,
    CC26XX_GPTIMER1B,
    CC26XX_GPTIMER2A,
    CC26XX_GPTIMER2B,
    CC26XX_GPTIMER3A,
    CC26XX_GPTIMER3B,
    CC26XX_GPTIMERPARTSCOUNT
} CC26XX_GPTimerName;

/*!
 *  @def    CC26XX_GPTimers
 *  @brief  Enum of GPTimers
 */
typedef enum CC26XX_GPTimers
{
    CC26XX_GPTIMER0 = 0,
    CC26XX_GPTIMER1,
    CC26XX_GPTIMER2,
    CC26XX_GPTIMER3,
    CC26XX_GPTIMERCOUNT
} CC26XX_GPTimers;

/*!
 *  @def    CC26XX_PWM
 *  @brief  Enum of PWM outputs on the board
 */
typedef enum CC26XX_PWM
{
    CC26XX_PWM0 = 0,
    CC26XX_PWM1,
    CC26XX_PWM2,
    CC26XX_PWM3,
    CC26XX_PWM4,
    CC26XX_PWM5,
    CC26XX_PWM6,
    CC26XX_PWM7,
    CC26XX_PWMCOUNT
} CC26XX_PWM;

/*!
 *  @def    CC26XX_ADCName
 *  @brief  Enum of ADCs
 */
typedef enum CC26XX_ADCName {
    CC26XX_ADC0 = 0,
    CC26XX_ADC1,
    CC26XX_ADC2,
    CC26XX_ADC3,
    CC26XX_ADC4,
#ifndef CC2650DK_4XS
    CC26XX_ADC5,
    CC26XX_ADC6,
    CC26XX_ADC7,
#endif
    CC26XX_ADCDCOUPL,
    CC26XX_ADCVSS,
    CC26XX_ADCVDDS,
    CC26XX_ADCCOUNT
} CC26XX_ADCName;

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H__ */
