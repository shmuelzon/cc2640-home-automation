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

// Position of motion sensor state in attribute array
#define MOTION_SENSOR_STATE_LEVEL_VALUE_IDX 2

#define MOTION_SENSOR_STATE_VALUE_LEN 1

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Motion Sensor service
CONST uint8_t motionSensorServiceUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(MOTION_SENSOR_SERV_UUID), HI_UINT16(MOTION_SENSOR_SERV_UUID)
};

// Motion Sensor state characteristic
CONST uint8_t motionSensorStateUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(MOTION_SENSOR_STATE_UUID), HI_UINT16(MOTION_SENSOR_STATE_UUID)
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
static CONST gattAttrType_t motionSensorService = { ATT_BT_UUID_SIZE,
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
        { ATT_BT_UUID_SIZE, motionSensorStateUUID },
        GATT_PERMIT_AUTHEN_READ,
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

static void MotionSensorNotify(uint16_t connHandle);
static void MotionSensorNotifyState(void);

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
          MotionSensorNotifyState();
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
  bStatus_t status = SUCCESS;

  // Make sure it's not a blob operation (no attributes in the profile are long)
  if (offset > 0)
    return ATT_ERR_ATTR_NOT_LONG;

  uint16_t uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);

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

  uint16_t uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);
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
 * @fn          MotionSensorNotify
 *
 * @brief       Send a notification of the motion sensor state characteristic.
 *
 * @param       connHandle - linkDB item
 *
 * @return      None.
 */
static void MotionSensorNotify(uint16_t connHandle)
{
  uint16_t value = GATTServApp_ReadCharCfg(connHandle,
    motionSensorStateClientCharCfg);

  if (value & GATT_CLIENT_CFG_NOTIFY)
  {
    attHandleValueNoti_t noti;

    noti.pValue = GATT_bm_alloc(connHandle, ATT_HANDLE_VALUE_NOTI,
      MOTION_SENSOR_STATE_VALUE_LEN, NULL);
    if (noti.pValue != NULL)
    {
      noti.handle =
        motionSensorAttrTbl[MOTION_SENSOR_STATE_LEVEL_VALUE_IDX].handle;
      noti.len = MOTION_SENSOR_STATE_VALUE_LEN;
      noti.pValue[0] = motionSensorState;

      if (GATT_Notification(connHandle, &noti, FALSE) != SUCCESS)
        GATT_bm_free((gattMsg_t *)&noti, ATT_HANDLE_VALUE_NOTI);
    }
  }
}

/*********************************************************************
 * @fn      MotionSensorNotifyState
 *
 * @brief   Send a notification of the motion sensor state
 *          characteristic if a connection is established.
 *
 * @return  None.
 */
static void MotionSensorNotifyState(void)
{
  uint8_t i;
  for (i = 0; i < linkDBNumConns; i++)
  {
    uint16_t connHandle = motionSensorStateClientCharCfg[i].connHandle;

    // Send notification to connected device.
    if (connHandle != INVALID_CONNHANDLE)
      MotionSensorNotify(connHandle);
  }
}

/*********************************************************************
*********************************************************************/
