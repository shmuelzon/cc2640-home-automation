#ifndef MOTIONSENSORSERVICE_H
#define MOTIONSENSORSERVICE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */


/*********************************************************************
 * CONSTANTS */


// Motion Sensor Service Get/Set Parameters
#define MOTION_SENSOR_PARAM_STATE              0

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
 * @fn      MotionSensor_AddService
 *
 * @brief   Initializes the Motion Sensor service by registering
 *          GATT attributes with the GATT server.
 *
 * @return  Success or Failure
 */
extern bStatus_t MotionSensor_AddService(void);

/*********************************************************************
 * @fn      MotionSensor_SetParameter
 *
 * @brief   Set a Motion Sensor Service parameter.
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
extern bStatus_t MotionSensor_SetParameter(uint8_t param, uint8_t len, void *value);

/*********************************************************************
 * @fn      MotionSensor_GetParameter
 *
 * @brief   Get a Motion Sensor parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to get.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
extern bStatus_t MotionSensor_GetParameter(uint8_t param, void *value);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* MOTIONSENSORSERVICE_H */
