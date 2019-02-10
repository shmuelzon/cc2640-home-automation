/** ============================================================================
 *  @file       SensorThermistor.h
 *
 *  @brief      Driver for reading an NTC 3950 100K thermistor.
 *
 *  ============================================================================
 */
#ifndef SENSOR_THERMISTOR_H
#define SENSOR_THERMISTOR_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "stdint.h"
#include "stdbool.h"

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * FUNCTIONS
 */
bool SensorThermistor_init(void);
void SensorThermistor_enable(bool enable);
bool SensorThermistor_read(int16_t *temp);
uint16_t SensorThermistor_measurementTime(void);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_THERMISTOR_H */
