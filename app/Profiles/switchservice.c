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

// Position of switch state in attribute array
#define SWITCH_STATE_LEVEL_VALUE_IDX 2

#define SWITCH_STATE_VALUE_LEN 1

// Toggle own relay UUID
#define SWITCH_TOGGLE_OWN_RELAY_UUID 0x2F12

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Switch service
CONST uint8_t switchServiceUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(SWITCH_SERV_UUID), HI_UINT16(SWITCH_SERV_UUID)
};

// Switch state characteristic
CONST uint8_t switchStateUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(SWITCH_STATE_UUID), HI_UINT16(SWITCH_STATE_UUID)
};

CONST uint8_t toggleOwnRelayUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(SWITCH_TOGGLE_OWN_RELAY_UUID), HI_UINT16(SWITCH_TOGGLE_OWN_RELAY_UUID)
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

// Toggle own relay configuration change callback.
static switchServiceToggleOwnRelayChangeCB_t switchServiceToggleOwnRelayChangeCB = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// Switch Service attribute.
static CONST gattAttrType_t switchService = { ATT_BT_UUID_SIZE, switchServiceUUID };

// Switch state characteristic.
static uint8_t switchStateProps = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Switch state value
static uint8_t switchState = 0xff;

// Switch state user description
uint8_t switchStateUserDesc[] = "Switch State";

static gattCharCfg_t *switchStateClientCharCfg;

// Toggle own relay characteristic.
static uint8_t toggleOwnRelayProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Toggle own relay value
static uint8_t toggleOwnRelay = 1;

// Toggle own relay user description
uint8_t toggleOwnRelayUserDesc[] = "Toggle own relay";

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
        { ATT_BT_UUID_SIZE, switchStateUUID },
        GATT_PERMIT_READ,
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

    // Toggle Own Relay Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &toggleOwnRelayProps
    },

      // Toggle Own Relay Value
      {
        { ATT_BT_UUID_SIZE, toggleOwnRelayUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        &toggleOwnRelay
      },

      // Toggle Own Relay User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        toggleOwnRelayUserDesc
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

static void SwitchNotify(uint16_t connHandle);
static void SwitchNotifyState(void);

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
          SwitchNotifyState();
        }
      }
      else
        ret = bleInvalidRange;
      break;

    case SWITCH_PARAM_TOGGLE_OWN_RELAY:
      if (len == sizeof(uint8_t))
        toggleOwnRelay = *((uint8_t*)value);
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

    case SWITCH_PARAM_TOGGLE_OWN_RELAY:
      *((uint8_t*)value) = toggleOwnRelay;
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return (ret);
}

/*********************************************************************
 * @fn      Switch_Setup
 *
 * @brief   Set up callback functions.
 *
 * @param   cCB - function to be called when ToggleOwnRelay has changed
 *
 * @return  none.
 */
void Switch_Setup(switchServiceToggleOwnRelayChangeCB_t cCB)
{
  switchServiceToggleOwnRelayChangeCB = cCB;
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
  bStatus_t status = SUCCESS;

  // Make sure it's not a blob operation (no attributes in the profile are long)
  if (offset > 0)
    return ATT_ERR_ATTR_NOT_LONG;

  uint16_t uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);

  if (uuid == SWITCH_STATE_UUID)
  {
    *pLen = 1;
    pValue[0] = switchState;
  }
  else if (uuid == SWITCH_TOGGLE_OWN_RELAY_UUID)
  {
    *pLen = 1;
    pValue[0] = toggleOwnRelay;
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

  uint16_t uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);
  switch (uuid)
  {
    case SWITCH_TOGGLE_OWN_RELAY_UUID:
      //Validate the value
      if (len != 1)
        status = ATT_ERR_INVALID_VALUE_SIZE;

      if (status == SUCCESS)
      {
        uint8 *pCurValue = (uint8 *)pAttr->pValue;

        /* Change the configuration only if the logical value changed */
        if (*pCurValue != !!(pValue[0]))
        {
          if (switchServiceToggleOwnRelayChangeCB)
            switchServiceToggleOwnRelayChangeCB();
          *pCurValue = !!pValue[0]; // Save the value
        }
      }
      break;

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
 * @fn          SwitchNotify
 *
 * @brief       Send a notification of the switch state characteristic.
 *
 * @param       connHandle - linkDB item
 *
 * @return      None.
 */
static void SwitchNotify(uint16_t connHandle)
{
  uint16_t value = GATTServApp_ReadCharCfg(connHandle,
    switchStateClientCharCfg);

  if (value & GATT_CLIENT_CFG_NOTIFY)
  {
    attHandleValueNoti_t noti;

    noti.pValue = GATT_bm_alloc(connHandle, ATT_HANDLE_VALUE_NOTI,
      SWITCH_STATE_VALUE_LEN, NULL);
    if (noti.pValue != NULL)
    {
      noti.handle = switchAttrTbl[SWITCH_STATE_LEVEL_VALUE_IDX].handle;
      noti.len = SWITCH_STATE_VALUE_LEN;
      noti.pValue[0] = switchState;

      if (GATT_Notification(connHandle, &noti, FALSE) != SUCCESS)
        GATT_bm_free((gattMsg_t *)&noti, ATT_HANDLE_VALUE_NOTI);
    }
  }
}

/*********************************************************************
 * @fn      SwitchNotifyState
 *
 * @brief   Send a notification of the switch state
 *          characteristic if a connection is established.
 *
 * @return  None.
 */
static void SwitchNotifyState(void)
{
  uint8_t i;
  for (i = 0; i < linkDBNumConns; i++)
  {
    uint16_t connHandle = switchStateClientCharCfg[i].connHandle;

    // Send notification to connected device.
    if (connHandle != INVALID_CONNHANDLE)
      SwitchNotify(connHandle);
  }
}

/*********************************************************************
*********************************************************************/
