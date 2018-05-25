#ifndef ENVIRONMENTALSENSINGSERVICE_H
#define ENVIRONMENTALSENSINGSERVICE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */


/*********************************************************************
 * CONSTANTS */
// Available services
#define ESS_SERVICE_LUMINANCE          (1 << 0)
#define ESS_SERVICE_PRESSURE           (1 << 1)
#define ESS_SERVICE_TEMPERATURE        (1 << 2)
#define ESS_SERVICE_HUMIDITY           (1 << 3)

// Service Get/Set Parameters
#define LUMINANCE_PARAM_VALUE          0
#define PRESSURE_PARAM_VALUE           1
#define TEMPERATURE_PARAM_VALUE        2
#define HUMIDITY_PARAM_VALUE           3

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */


/*********************************************************************
 * API FUNCTIONS
 */

/*********************************************************************
 * @fn      EnvironmentalSensing_AddService
 *
 * @brief   Initializes the Environmental Sensing service by registering
 *          GATT attributes with the GATT server.
 *
 * @return  Success or Failure
 */
extern bStatus_t EnvironmentalSensing_AddService(uint32_t services);

/*********************************************************************
 * @fn      EnvironmentalSensing_SetParameter
 *
 * @brief   Set an Environmental Sensing Service parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to right
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
extern bStatus_t EnvironmentalSensing_SetParameter(uint8_t param, uint8_t len,
  void *value);

/*********************************************************************
 * @fn      EnvironmentalSensing_getParameter
 *
 * @brief   Get an Environmental Sensing parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to get.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
extern bStatus_t EnvironmentalSensing_getParameter(uint8_t param, void *value);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* ENVIRONMENTALSENSINGSERVICE_H */
