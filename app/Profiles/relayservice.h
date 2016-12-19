#ifndef RELAYSERVICE_H
#define RELAYSERVICE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS */


// Relay Service Get/Set Parameters
#define RELAY_PARAM_STATE               0

/*********************************************************************
 * TYPEDEFS
 */

// Relay state change function
typedef void (*relayServiceStateChangeCB_t)(void);

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
 * @fn      Relay_AddService
 *
 * @brief   Initializes the Relay service by registering
 *          GATT attributes with the GATT server.
 *
 * @return  Success or Failure
 */
extern bStatus_t Relay_AddService(void);

/*********************************************************************
 * @fn      Relay_SetParameter
 *
 * @brief   Set a Relay Service parameter.
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
extern bStatus_t Relay_SetParameter(uint8_t param, uint8_t len, void *value);

/*********************************************************************
 * @fn      Relay_GetParameter
 *
 * @brief   Get a Relay parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to get.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
extern bStatus_t Relay_GetParameter(uint8_t param, void *value);

/*********************************************************************
 * @fn      Relay_Setup
 *
 * @brief   Set up application callback functions.
 *
 * @param   cCB - relay state change callback
 *
 * @return  none.
 */
extern void Relay_Setup(relayServiceStateChangeCB_t cCB);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* RELAYSERVICE_H */
