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

#include "contactsensorservice.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Contact Sensor Profile Service UUID
#define CONTACT_SENSOR_SERV_UUID 0x2F20

// Contact Sensor state UUID
#define CONTACT_SENSOR_STATE_UUID 0x2F21

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
// Contact Sensor service
CONST uint8_t contactSensorServiceUUID[HA_UUID_SIZE] =
{
  HA_UUID(CONTACT_SENSOR_SERV_UUID)
};

// Contact Sensor state characteristic
CONST uint8_t contactSensorStateUUID[HA_UUID_SIZE] =
{
  HA_UUID(CONTACT_SENSOR_STATE_UUID)
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

// Contact Sensor Service attribute.
static CONST gattAttrType_t contactSensorService = { HA_UUID_SIZE,
  contactSensorServiceUUID };

// Contact Sensor state characteristic.
static uint8_t contactSensorStateProps = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Contact Sensor state value
static uint8_t contactSensorState = 0xff;

// Contact Sensor state user description
uint8_t contactSensorStateUserDesc[] = "Contact Sensor State";

static gattCharCfg_t *contactSensorStateClientCharCfg;

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t contactSensorAttrTbl[] =
{
  // Contact Sensor Service
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8_t *)&contactSensorService          /* pValue */
  },

    // Contact Sensor State Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &contactSensorStateProps
    },

      // Contact Sensor State Value
      {
        { HA_UUID_SIZE, contactSensorStateUUID },
#ifndef DISABLE_AUTHENTICATION
        GATT_PERMIT_AUTHEN_READ,
#else
        GATT_PERMIT_READ,
#endif
        0,
        &contactSensorState
      },

      // Contact Sensor State Client Characteristic Configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
#ifndef DISABLE_AUTHENTICATION
        GATT_PERMIT_READ | GATT_PERMIT_AUTHEN_WRITE,
#else
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
#endif
        0,
        (uint8_t *)&contactSensorStateClientCharCfg
      },

      // Contact Sensor State User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        contactSensorStateUserDesc
      },
};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t ContactSensorReadAttrCB(uint16_t connHandle,
  gattAttribute_t *pAttr, uint8_t *pValue, uint16_t *pLen, uint16_t offset,
  uint16_t maxLen, uint8_t method);
static bStatus_t ContactSensorWriteAttrCB(uint16_t connHandle,
  gattAttribute_t *pAttr, uint8_t *pValue, uint16_t len, uint16_t offset,
  uint8_t method);

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Contact Sensor Service Callbacks
CONST gattServiceCBs_t contactSensorCBs =
{
  ContactSensorReadAttrCB,  // Read callback function pointer
  ContactSensorWriteAttrCB, // Write callback function pointer
  NULL                      // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      ContactSensor_AddService
 *
 * @brief   Initializes the Contact Sensor service by registering
 *          GATT attributes with the GATT server.
 *
 * @return  Success or Failure
 */
bStatus_t ContactSensor_AddService(void)
{
  // Allocate Client Characteristic Configuration table
  contactSensorStateClientCharCfg = (gattCharCfg_t *)ICall_malloc(
    sizeof(gattCharCfg_t) * linkDBNumConns);

  if (contactSensorStateClientCharCfg == NULL)
    return bleMemAllocError;

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg(INVALID_CONNHANDLE, contactSensorStateClientCharCfg);

  // Register GATT attribute list and CBs with GATT Server App
  return GATTServApp_RegisterService(contactSensorAttrTbl,
    GATT_NUM_ATTRS(contactSensorAttrTbl), GATT_MAX_ENCRYPT_KEY_SIZE,
    &contactSensorCBs);
}

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
bStatus_t ContactSensor_SetParameter(uint8_t param, uint8_t len, void *value)
{
  bStatus_t ret = SUCCESS;

  switch (param)
  {
    case CONTACT_SENSOR_PARAM_STATE:
      if (len == sizeof(uint8_t))
      {
        uint8_t state = *((uint8_t*)value);

        if (state != contactSensorState)
        {
          contactSensorState = state;
          ret = GATTServApp_ProcessCharCfg(contactSensorStateClientCharCfg,
            &contactSensorState, NOTIFY_AUTH, contactSensorAttrTbl,
            GATT_NUM_ATTRS(contactSensorAttrTbl), INVALID_TASK_ID,
            ContactSensorReadAttrCB);
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
bStatus_t ContactSensor_GetParameter(uint8_t param, void *value)
{
  bStatus_t ret = SUCCESS;

  switch (param)
  {
    case CONTACT_SENSOR_PARAM_STATE:
      *((uint8_t*)value) = contactSensorState;
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return (ret);
}

/*********************************************************************
 * @fn          ContactSensorReadAttrCB
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
static bStatus_t ContactSensorReadAttrCB(uint16_t connHandle,
  gattAttribute_t *pAttr, uint8_t *pValue, uint16_t *pLen, uint16_t offset,
  uint16_t maxLen, uint8_t method)
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

  if (uuid == CONTACT_SENSOR_STATE_UUID)
  {
    *pLen = 1;
    pValue[0] = contactSensorState;
  }
  else
    status = ATT_ERR_ATTR_NOT_FOUND;

  return (status);
}

/*********************************************************************
 * @fn      ContactSensorWriteAttrCB
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
static bStatus_t ContactSensorWriteAttrCB(uint16_t connHandle,
  gattAttribute_t *pAttr, uint8_t *pValue, uint16_t len, uint16_t offset,
  uint8_t method)
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
