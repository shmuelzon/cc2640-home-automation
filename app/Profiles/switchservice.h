#ifndef SWITCHSERVICE_H
#define SWITCHSERVICE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

#include <ti/drivers/PIN.h>

/*********************************************************************
 * CONSTANTS */


// Switch Service Get/Set Parameters
#define SWITCH_PARAM_STATE              0
#define SWITCH_PARAM_TOGGLE_OWN_RELAY   1

/*********************************************************************
 * TYPEDEFS
 */

// Toggle own relay configuration change function
typedef void (*switchServiceToggleOwnRelayChangeCB_t)(void);

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
 * @fn      Switch_AddService
 *
 * @brief   Initializes the Switch service by registering
 *          GATT attributes with the GATT server.
 *
 * @return  Success or Failure
 */
extern bStatus_t Switch_AddService(void);

/*********************************************************************
 * @fn      Switch_SetParameter
 *
 * @brief   Set a Switch Service parameter.
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
extern bStatus_t Switch_SetParameter(uint8_t param, uint8_t len, void *value);

/*********************************************************************
 * @fn      Switch_GetParameter
 *
 * @brief   Get a Switch parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to get.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
extern bStatus_t Switch_GetParameter(uint8_t param, void *value);

/*********************************************************************
 * @fn      Switch_Setup
 *
 * @brief   Set up callback functions.
 *
 * @param   cCB - function to be called when ToggleOwnRelay has changed
 *
 * @return  none.
 */
extern void Switch_Setup(switchServiceToggleOwnRelayChangeCB_t cCB);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* SWITCHSERVICE_H */
