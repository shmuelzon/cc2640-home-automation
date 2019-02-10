/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gatt_profile_uuid.h"
#include "gattservapp.h"
#include "ha_util.h"

#include "switchservice.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Switch Profile Service UUID
#define SWITCH_SERV_UUID 0x2F10

// Switch state UUID
#define SWITCH_STATE_UUID 0x2F11

#ifndef DISABLE_AUTHENTICATION
#define NOTIFY_AUTH TRUE
#else
#define NOTIFY_AUTH FALSE
#endif

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Switch service
CONST uint8_t switchServiceUUID[HA_UUID_SIZE] =
{
  HA_UUID(SWITCH_SERV_UUID)
};

// Switch state characteristic
CONST uint8_t switchStateUUID[HA_UUID_SIZE] =
{
  HA_UUID(SWITCH_STATE_UUID)
};

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

/*********************************************************************
 * Profile Attributes - variables
 */

// Switch Service attribute.
static CONST gattAttrType_t switchService = { HA_UUID_SIZE, switchServiceUUID };

// Switch state characteristic.
static uint8_t switchStateProps = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Switch state value
static uint8_t switchState = 0xff;

// Switch state user description
uint8_t switchStateUserDesc[] = "Switch State";

static gattCharCfg_t *switchStateClientCharCfg;

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t switchAttrTbl[] =
{
  // Switch Service
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8_t *)&switchService                 /* pValue */
  },

    // Switch State Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &switchStateProps
    },

      // Switch State Value
      {
        { HA_UUID_SIZE, switchStateUUID },
#ifndef DISABLE_AUTHENTICATION
        GATT_PERMIT_AUTHEN_READ,
#else
        GATT_PERMIT_READ,
#endif
        0,
        &switchState
      },

      // Switch State Client Characteristic Configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8_t *)&switchStateClientCharCfg
      },

      // Switch State User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        switchStateUserDesc
      },
};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t SwitchReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
  uint8_t *pValue, uint16_t *pLen, uint16_t offset, uint16_t maxLen,
  uint8_t method);
static bStatus_t SwitchWriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
  uint8_t *pValue, uint16_t len, uint16_t offset, uint8_t method);

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Switch Service Callbacks
CONST gattServiceCBs_t switchCBs =
{
  SwitchReadAttrCB,  // Read callback function pointer
  SwitchWriteAttrCB, // Write callback function pointer
  NULL               // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Switch_AddService
 *
 * @brief   Initializes the Switch service by registering
 *          GATT attributes with the GATT server.
 *
 * @return  Success or Failure
 */
bStatus_t Switch_AddService(void)
{
  // Allocate Client Characteristic Configuration table
  switchStateClientCharCfg = (gattCharCfg_t *)ICall_malloc(
    sizeof(gattCharCfg_t) * linkDBNumConns);

  if (switchStateClientCharCfg == NULL)
    return bleMemAllocError;

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg(INVALID_CONNHANDLE, switchStateClientCharCfg);

  // Register GATT attribute list and CBs with GATT Server App
  return GATTServApp_RegisterService(switchAttrTbl,
    GATT_NUM_ATTRS(switchAttrTbl), GATT_MAX_ENCRYPT_KEY_SIZE, &switchCBs);
}

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
bStatus_t Switch_SetParameter(uint8_t param, uint8_t len, void *value)
{
  bStatus_t ret = SUCCESS;

  switch (param)
  {
    case SWITCH_PARAM_STATE:
      if (len == sizeof(uint8_t))
      {
        uint8_t state = *((uint8_t*)value);

        if (state != switchState)
        {
          switchState = state;
          ret = GATTServApp_ProcessCharCfg(switchStateClientCharCfg,
            &switchState, NOTIFY_AUTH, switchAttrTbl,
            GATT_NUM_ATTRS(switchAttrTbl), INVALID_TASK_ID,
            SwitchReadAttrCB);
        }
      }
      else
        ret = bleInvalidRange;
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return ret;
}

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
bStatus_t Switch_GetParameter(uint8_t param, void *value)
{
  bStatus_t ret = SUCCESS;

  switch (param)
  {
    case SWITCH_PARAM_STATE:
      *((uint8_t*)value) = switchState;
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return (ret);
}

/*********************************************************************
 * @fn          switchReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message
 *
 * @return      SUCCESS, blePending or Failure
 */
static bStatus_t SwitchReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
  uint8_t *pValue, uint16_t *pLen, uint16_t offset, uint16_t maxLen,
  uint8_t method)
{
  uint16_t uuid;
  bStatus_t status = SUCCESS;

  // Make sure it's not a blob operation (no attributes in the profile are long)
  if (offset > 0)
    return ATT_ERR_ATTR_NOT_LONG;

  if (utilExtractUuid16(pAttr, &uuid) == FAILURE)
  {
    // Invalid handle
    *pLen = 0;
    return ATT_ERR_INVALID_HANDLE;
  }

  if (uuid == SWITCH_STATE_UUID)
  {
    *pLen = 1;
    pValue[0] = switchState;
  }
  else
    status = ATT_ERR_ATTR_NOT_FOUND;

  return (status);
}

/*********************************************************************
 * @fn      switchWriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message
 *
 * @return  SUCCESS, blePending or Failure
 */
static bStatus_t SwitchWriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
  uint8_t *pValue, uint16_t len, uint16_t offset, uint8_t method)
{
  bStatus_t status = SUCCESS;
  uint16_t uuid;

  if (utilExtractUuid16(pAttr, &uuid) == FAILURE)
  {
    // Invalid handle
    return ATT_ERR_INVALID_HANDLE;
  }

  switch (uuid)
  {
    case GATT_CLIENT_CHAR_CFG_UUID:
      status = GATTServApp_ProcessCCCWriteReq(connHandle, pAttr, pValue, len,
        offset, GATT_CLIENT_CFG_NOTIFY);
      break;

    default:
      status = ATT_ERR_ATTR_NOT_FOUND;
      break;
  }

  return (status);
}

/*********************************************************************
*********************************************************************/
