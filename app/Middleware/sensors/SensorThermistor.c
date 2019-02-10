/** ============================================================================
 *  @file       SensorThermistor.c
 *
 *  @brief      Driver for reading an NTC 3950 100K thermistor.
 *
 *  ============================================================================
 */

/* -----------------------------------------------------------------------------
*  Includes
* ------------------------------------------------------------------------------
*/
#include "board.h"
#include "SensorThermistor.h"
#include <ti/drivers/ADC.h>
#include "math.h"

/* -----------------------------------------------------------------------------
 *  Constants and macros
 * -----------------------------------------------------------------------------
 */

/* Thermistor type */
#define THERMISTOR_NOMINAL_RESISTANCE   100000 // 100K Thermistor
#define THERMISTOR_NOMINAL_TEMPERATURE  25
#define THERMISTOR_BETA_COEFFICIENT     3950
#define THERMISTOR_SERIES_RESISTOR      100000 // Bottom resistor in the voltage divider pair

const PIN_Config thermistorGpioInitTable[] = {
#if Board_THERMISTOR_POWER != PIN_UNASSIGNED
  Board_THERMISTOR_POWER | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
#endif
    PIN_TERMINATE /* Terminate list */
};

/* -----------------------------------------------------------------------------
 * Local variables
 * -----------------------------------------------------------------------------
 */

// GPIOs
#if Board_THERMISTOR_POWER != PIN_UNASSIGNED
static PIN_State thermistorPinState;
static PIN_Handle thermistorPinHandle;
#endif

// ADC
static ADC_Handle thermistorAdc;

/* -----------------------------------------------------------------------------
 *  Public functions
 * -----------------------------------------------------------------------------
 */

/*******************************************************************************
 * @fn          SensorThermistor_init
 *
 * @brief       Initialize the thermistor driver
 *
 * @return      none
 ******************************************************************************/
bool SensorThermistor_init(void)
{
  ADC_Params thermistorParams;

#if Board_THERMISTOR_POWER != PIN_UNASSIGNED
  // Setup GPIO
  thermistorPinHandle = PIN_open(&thermistorPinState, thermistorGpioInitTable);
  SensorThermistor_enable(false);
#endif

  // Setup ADC
  ADC_Params_init(&thermistorParams);
  if ((thermistorAdc = ADC_open(Board_THERMISTOR, &thermistorParams)) == NULL)
    return false;

  return true;
}

/*******************************************************************************
 * @fn          SensorThermistor_enable
 *
 * @brief       Turn the thermistor on/off
 *
 * @return      none
 ******************************************************************************/
void SensorThermistor_enable(bool enable)
{
#if Board_THERMISTOR_POWER != PIN_UNASSIGNED
  PIN_setOutputValue(thermistorPinHandle, Board_THERMISTOR_POWER, enable);
#endif
}

/*******************************************************************************
 * @fn          SensorThermistor_read
 *
 * @brief       Read the thermistor temperature value
 *
 * @param       temp - pointer for storing the temperature value
 *
 * @return      true if valid data
 ******************************************************************************/
bool SensorThermistor_read(int16_t *temp)
{
  uint16_t adc;
  float resistance, steinhart;

  // Read ADC Value
  if (ADC_convert(thermistorAdc, &adc) != ADC_STATUS_SUCCESS)
    return false;

  // Convert to resistance
  resistance = THERMISTOR_SERIES_RESISTOR * ((4095.0 / (float)adc) - 1.0);

  // Calculate temperature (Kelvin)
  // (1 / T) = (1 / T0) + (1 / B) * ln(R / R0)
  steinhart = 1.0 / ((1.0 / (THERMISTOR_NOMINAL_TEMPERATURE + 273.15)) +
    logf(resistance / THERMISTOR_NOMINAL_RESISTANCE) / THERMISTOR_BETA_COEFFICIENT);
  // Convert to Celsius
  steinhart -= 273.15;

  // Convert to int
  *temp = (int16_t)(steinhart * 100);

  return true;
}

/*******************************************************************************
 * @fn          SensorThermistor_measurementTime
 *
 * @brief       Return the length of measurement time
 *
 * @return      Number of milliseconds for measurement
 *
 ******************************************************************************/
uint16_t SensorThermistor_measurementTime(void)
{
  return 150;
}
