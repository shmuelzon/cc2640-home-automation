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

#include "environmentalsensingservice.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Environmental Sensing Profile Service UUID
#define ENVIRONMENTAL_SENSING_SERV_UUID 0x181A

// Luminance UUID
#define LUMINANCE_UUID 0x2F41
#define LUMINANCE_VALUE_LEN 4

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Environmental Sensing service
CONST uint8_t environmentalSensingServiceUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(ENVIRONMENTAL_SENSING_SERV_UUID),
  HI_UINT16(ENVIRONMENTAL_SENSING_SERV_UUID)
};

// Luminance characteristic
CONST uint8_t luminanceUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(LUMINANCE_UUID), HI_UINT16(LUMINANCE_UUID)
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

// Environmental Sensing Service attribute.
static CONST gattAttrType_t environmentalSensingService = { ATT_BT_UUID_SIZE,
  environmentalSensingServiceUUID };

// Luminance characteristic.
static uint8_t luminanceProps = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Luminance value
static uint32_t luminanceValue = 0;

// Luminance user description
uint8_t luminanceUserDesc[] = "Luminance";

static gattCharCfg_t *luminanceClientCharCfg;

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t environmentalSensingAttrTbl[5] = // Service + 1 char
{
  // Environmental Sensing Service
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8_t *)&environmentalSensingService   /* pValue */
  }
};
static size_t environmentalSensingAttrTblSize = 1;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t EnvironmentalSensingReadAttrCB(uint16_t connHandle,
  gattAttribute_t *pAttr, uint8_t *pValue, uint16_t *pLen, uint16_t offset,
  uint16_t maxLen, uint8_t method);
static bStatus_t EnvironmentalSensingWriteAttrCB(uint16_t connHandle,
  gattAttribute_t *pAttr, uint8_t *pValue, uint16_t len, uint16_t offset,
  uint8_t method);

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Environmental Sensing Service Callbacks
CONST gattServiceCBs_t environmentalSensingCBs =
{
  EnvironmentalSensingReadAttrCB,  // Read callback function pointer
  EnvironmentalSensingWriteAttrCB, // Write callback function pointer
  NULL                      // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      EnvironmentalSensing_AddService
 *
 * @brief   Initializes the Environmental Sensing service by registering
 *          GATT attributes with the GATT server.
 *
 * @return  Success or Failure
 */
bStatus_t EnvironmentalSensing_AddService(uint32_t services)
{
  if (services & ESS_SERVICE_LUMINANCE)
  {
    gattAttribute_t luminance[] = {
      // Luminance Declaration
      {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &luminanceProps
      },

        // Luminance Value
        {
          { ATT_BT_UUID_SIZE, luminanceUUID },
#ifndef DISABLE_AUTHENTICATION
          GATT_PERMIT_AUTHEN_READ,
#else
          GATT_PERMIT_READ,
#endif
          0,
          (uint8_t *)&luminanceValue
        },

        // Luminance Client Characteristic Configuration
        {
          { ATT_BT_UUID_SIZE, clientCharCfgUUID },
          GATT_PERMIT_READ | GATT_PERMIT_WRITE,
          0,
          (uint8_t *)&luminanceClientCharCfg
        },

        // Luminance User Description
        {
          { ATT_BT_UUID_SIZE, charUserDescUUID },
          GATT_PERMIT_READ,
          0,
          luminanceUserDesc
        },
    };
    size_t numAttr = GATT_NUM_ATTRS(luminance);

    // Add luminance characteristic
    memcpy(&environmentalSensingAttrTbl[environmentalSensingAttrTblSize],
      &luminance, numAttr * sizeof(*environmentalSensingAttrTbl));
    environmentalSensingAttrTblSize += numAttr;

    // Allocate Client Characteristic Configuration table
    luminanceClientCharCfg = (gattCharCfg_t *)ICall_malloc(
      sizeof(gattCharCfg_t) * linkDBNumConns);

    if (luminanceClientCharCfg == NULL)
      return bleMemAllocError;

    // Initialize Client Characteristic Configuration attributes
    GATTServApp_InitCharCfg(INVALID_CONNHANDLE, luminanceClientCharCfg);
  }

  // Register GATT attribute list and CBs with GATT Server App
  return GATTServApp_RegisterService(environmentalSensingAttrTbl,
    environmentalSensingAttrTblSize, GATT_MAX_ENCRYPT_KEY_SIZE,
    &environmentalSensingCBs);
}

/*********************************************************************
 * @fn      EnvironmentalSensing_SetParameter
 *
 * @brief   Set an Environmental Sensing Service parameter.
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
bStatus_t EnvironmentalSensing_SetParameter(uint8_t param, uint8_t len, void *value)
{
  bStatus_t ret = SUCCESS;

  switch (param)
  {
    case LUMINANCE_PARAM_VALUE:
      if (len == sizeof(uint32_t))
      {
        uint32_t val = *((uint32_t*)value);

        if (val != luminanceValue)
        {
          luminanceValue = val;
          // See if Notification has been enabled
          ret = GATTServApp_ProcessCharCfg(luminanceClientCharCfg, (uint8_t *)&luminanceValue, FALSE,
            environmentalSensingAttrTbl, environmentalSensingAttrTblSize,
            INVALID_TASK_ID, EnvironmentalSensingReadAttrCB);
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
 * @fn      EnvironmentalSensing_GetParameter
 *
 * @brief   Get an Environmental Sensing parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to get.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t EnvironmentalSensing_GetParameter(uint8_t param, void *value)
{
  bStatus_t ret = SUCCESS;

  switch (param)
  {
  case LUMINANCE_PARAM_VALUE:
    *((uint32_t*)value) = luminanceValue;
    break;
  default:
    ret = INVALIDPARAMETER;
    break;
  }

  return (ret);
}

/*********************************************************************
 * @fn          EnvironmentalSensingReadAttrCB
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
static bStatus_t EnvironmentalSensingReadAttrCB(uint16_t connHandle,
  gattAttribute_t *pAttr, uint8_t *pValue, uint16_t *pLen, uint16_t offset,
  uint16_t maxLen, uint8_t method)
{
  bStatus_t status = SUCCESS;

  // Make sure it's not a blob operation (no attributes in the profile are long)
  if (offset > 0)
    return ATT_ERR_ATTR_NOT_LONG;

  uint16_t uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);

  switch (uuid)
  {
    case LUMINANCE_UUID:
      *pLen = LUMINANCE_VALUE_LEN;
      memcpy(pValue, pAttr->pValue, LUMINANCE_VALUE_LEN);
      break;
    default:
      *pLen = 0;
      status = ATT_ERR_ATTR_NOT_FOUND;
      break;
    }

  return (status);
}

/*********************************************************************
 * @fn      EnvironmentalSensingWriteAttrCB
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
static bStatus_t EnvironmentalSensingWriteAttrCB(uint16_t connHandle,
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
*********************************************************************/
