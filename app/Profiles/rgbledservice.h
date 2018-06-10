#ifndef RGBLEDSERVICE_H
#define RGBLEDSERVICE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS */


// RGB LED Service Get/Set Parameters
#define RGB_LED_PARAM_COLOR               0

/*********************************************************************
 * TYPEDEFS
 */

// RGB LED color change function
typedef void (*rgbLedServiceColorChangeCB_t)(void);

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
 * @fn      RGBLED_AddService
 *
 * @brief   Initializes the RGB LED service by registering
 *          GATT attributes with the GATT server.
 *
 * @return  Success or Failure
 */
extern bStatus_t RGBLED_AddService(void);

/*********************************************************************
 * @fn      RGBLED_SetParameter
 *
 * @brief   Set a RGB LED Service parameter.
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
extern bStatus_t RGBLED_SetParameter(uint8_t param, uint8_t len, void *value);

/*********************************************************************
 * @fn      RGBLED_GetParameter
 *
 * @brief   Get a RGB LED parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to get.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
extern bStatus_t RGBLED_GetParameter(uint8_t param, void *value);

/*********************************************************************
 * @fn      RGBLED_Setup
 *
 * @brief   Set up application callback functions.
 *
 * @param   cCB - RGB LED color change callback
 *
 * @return  none.
 */
extern void RGBLED_Setup(rgbLedServiceColorChangeCB_t cCB);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* RGBLEDSERVICE_H */
