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

#include "motionsensorservice.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Motion Sensor Profile Service UUID
#define MOTION_SENSOR_SERV_UUID 0x2F30

// Motion Sensor state UUID
#define MOTION_SENSOR_STATE_UUID 0x2F31

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
// Motion Sensor service
CONST uint8_t motionSensorServiceUUID[HA_UUID_SIZE] =
{
  HA_UUID(MOTION_SENSOR_SERV_UUID)
};

// Motion Sensor state characteristic
CONST uint8_t motionSensorStateUUID[HA_UUID_SIZE] =
{
  HA_UUID(MOTION_SENSOR_STATE_UUID)
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

// Motion Sensor Service attribute.
static CONST gattAttrType_t motionSensorService = { HA_UUID_SIZE,
  motionSensorServiceUUID };

// Motion Sensor state characteristic.
static uint8_t motionSensorStateProps = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Motion Sensor state value
static uint8_t motionSensorState = 0xff;

// Motion Sensor state user description
uint8_t motionSensorStateUserDesc[] = "Motion Sensor State";

static gattCharCfg_t *motionSensorStateClientCharCfg;

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t motionSensorAttrTbl[] =
{
  // Motion Sensor Service
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8_t *)&motionSensorService          /* pValue */
  },

    // Motion Sensor State Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &motionSensorStateProps
    },

      // Motion Sensor State Value
      {
        { HA_UUID_SIZE, motionSensorStateUUID },
#ifndef DISABLE_AUTHENTICATION
        GATT_PERMIT_AUTHEN_READ,
#else
        GATT_PERMIT_READ,
#endif
        0,
        &motionSensorState
      },

      // Motion Sensor State Client Characteristic Configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8_t *)&motionSensorStateClientCharCfg
      },

      // Motion Sensor State User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        motionSensorStateUserDesc
      },
};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t MotionSensorReadAttrCB(uint16_t connHandle,
  gattAttribute_t *pAttr, uint8_t *pValue, uint16_t *pLen, uint16_t offset,
  uint16_t maxLen, uint8_t method);
static bStatus_t MotionSensorWriteAttrCB(uint16_t connHandle,
  gattAttribute_t *pAttr, uint8_t *pValue, uint16_t len, uint16_t offset,
  uint8_t method);

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Motion Sensor Service Callbacks
CONST gattServiceCBs_t motionSensorCBs =
{
  MotionSensorReadAttrCB,  // Read callback function pointer
  MotionSensorWriteAttrCB, // Write callback function pointer
  NULL                      // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      MotionSensor_AddService
 *
 * @brief   Initializes the Motion Sensor service by registering
 *          GATT attributes with the GATT server.
 *
 * @return  Success or Failure
 */
bStatus_t MotionSensor_AddService(void)
{
  // Allocate Client Characteristic Configuration table
  motionSensorStateClientCharCfg = (gattCharCfg_t *)ICall_malloc(
    sizeof(gattCharCfg_t) * linkDBNumConns);

  if (motionSensorStateClientCharCfg == NULL)
    return bleMemAllocError;

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg(INVALID_CONNHANDLE, motionSensorStateClientCharCfg);

  // Register GATT attribute list and CBs with GATT Server App
  return GATTServApp_RegisterService(motionSensorAttrTbl,
    GATT_NUM_ATTRS(motionSensorAttrTbl), GATT_MAX_ENCRYPT_KEY_SIZE,
    &motionSensorCBs);
}

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
bStatus_t MotionSensor_SetParameter(uint8_t param, uint8_t len, void *value)
{
  bStatus_t ret = SUCCESS;

  switch (param)
  {
    case MOTION_SENSOR_PARAM_STATE:
      if (len == sizeof(uint8_t))
      {
        uint8_t state = *((uint8_t*)value);

        if (state != motionSensorState)
        {
          motionSensorState = state;
          ret = GATTServApp_ProcessCharCfg(motionSensorStateClientCharCfg,
            &motionSensorState, NOTIFY_AUTH, motionSensorAttrTbl,
            GATT_NUM_ATTRS(motionSensorAttrTbl), INVALID_TASK_ID,
            MotionSensorReadAttrCB);
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
bStatus_t MotionSensor_GetParameter(uint8_t param, void *value)
{
  bStatus_t ret = SUCCESS;

  switch (param)
  {
    case MOTION_SENSOR_PARAM_STATE:
      *((uint8_t*)value) = motionSensorState;
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return (ret);
}

/*********************************************************************
 * @fn          MotionSensorReadAttrCB
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
static bStatus_t MotionSensorReadAttrCB(uint16_t connHandle,
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

  if (uuid == MOTION_SENSOR_STATE_UUID)
  {
    *pLen = 1;
    pValue[0] = motionSensorState;
  }
  else
    status = ATT_ERR_ATTR_NOT_FOUND;

  return (status);
}

/*********************************************************************
 * @fn      MotionSensorWriteAttrCB
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
static bStatus_t MotionSensorWriteAttrCB(uint16_t connHandle,
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
