#ifndef CONTACTSENSORSERVICE_H
#define CONTACTSENSORSERVICE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */


/*********************************************************************
 * CONSTANTS */


// Contact Sensor Service Get/Set Parameters
#define CONTACT_SENSOR_PARAM_STATE              0

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
 * @fn      ContactSensor_AddService
 *
 * @brief   Initializes the Contact Sensor service by registering
 *          GATT attributes with the GATT server.
 *
 * @return  Success or Failure
 */
extern bStatus_t ContactSensor_AddService(void);

/*********************************************************************
 * @fn      ContactSensor_SetParameter
 *
 * @brief   Set a Contact Sensor Service parameter.
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
extern bStatus_t ContactSensor_SetParameter(uint8_t param, uint8_t len, void *value);

/*********************************************************************
 * @fn      ContactSensor_GetParameter
 *
 * @brief   Get a Contact Sensor parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to get.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
extern bStatus_t ContactSensor_GetParameter(uint8_t param, void *value);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* CONTACTSENSORSERVICE_H */
