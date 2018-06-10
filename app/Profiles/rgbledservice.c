/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include "board.h"
#include "bcomdef.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gatt_profile_uuid.h"
#include "gattservapp.h"

#include "rgbledservice.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// RGB LED Profile Service UUID
#define RGB_LED_SERV_UUID 0x2F50

// RGB LED color UUID
#define RGB_LED_COLOR_UUID 0x2F51

#define RGB_LED_COLOR_VALUE_LEN (3 * Board_RGB_NUM_LEDS)

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// RGB LED service
CONST uint8_t rgbLedServiceUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(RGB_LED_SERV_UUID), HI_UINT16(RGB_LED_SERV_UUID)
};

// RGB LED color characteristic
CONST uint8_t rgbLedColorUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(RGB_LED_COLOR_UUID), HI_UINT16(RGB_LED_COLOR_UUID)
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

// RGB LED color change callback.
static rgbLedServiceColorChangeCB_t rgbLedServiceColorChangeCB = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// RGB LED Service attribute.
static CONST gattAttrType_t rgbLedService = { ATT_BT_UUID_SIZE, rgbLedServiceUUID };

// RGB LED color characteristic.
static uint8_t rgbLedColorProps = GATT_PROP_READ | GATT_PROP_WRITE | GATT_PROP_NOTIFY;

// RGB LED color value
static uint8_t rgbLedColor[RGB_LED_COLOR_VALUE_LEN];

// RGB LED color user description
uint8_t rgbLedColorUserDesc[] = "RGB LED Color";

static gattCharCfg_t *rgbLedColorClientCharCfg;

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t rgbLedAttrTbl[] =
{
  // RGB LED Service
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8_t *)&rgbLedService                  /* pValue */
  },

    // RGB LED Color Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &rgbLedColorProps
    },

      // RGB LED Color Value
      {
        { ATT_BT_UUID_SIZE, rgbLedColorUUID },
#ifndef DISABLE_AUTHENTICATION
        GATT_PERMIT_AUTHEN_READ | GATT_PERMIT_AUTHEN_WRITE,
#else
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
#endif
        0,
        rgbLedColor
      },

      // RGB LED Color Client Characteristic Configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8_t *)&rgbLedColorClientCharCfg
      },

      // RGB LED Color User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        rgbLedColorUserDesc
      },
};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t RGBLEDReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
  uint8_t *pValue, uint16_t *pLen, uint16_t offset, uint16_t maxLen,
  uint8_t method);
static bStatus_t RGBLEDWriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
  uint8_t *pValue, uint16_t len, uint16_t offset, uint8_t method);

/*********************************************************************
 * PROFILE CALLBACKS
 */
// RGB LED Service Callbacks
CONST gattServiceCBs_t rgbLedCBs =
{
  RGBLEDReadAttrCB,  // Read callback function pointer
  RGBLEDWriteAttrCB, // Write callback function pointer
  NULL              // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      RGBLED_AddService
 *
 * @brief   Initializes the RGB LED service by registering
 *          GATT attributes with the GATT server.
 *
 * @return  Success or Failure
 */
bStatus_t RGBLED_AddService(void)
{
  // Allocate Client Characteristic Configuration table
  rgbLedColorClientCharCfg = (gattCharCfg_t *)ICall_malloc(
    sizeof(gattCharCfg_t) * linkDBNumConns);

  if (rgbLedColorClientCharCfg == NULL)
    return bleMemAllocError;

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg(INVALID_CONNHANDLE, rgbLedColorClientCharCfg);

  // Register GATT attribute list and CBs with GATT Server App
  return GATTServApp_RegisterService(rgbLedAttrTbl,
    GATT_NUM_ATTRS(rgbLedAttrTbl), GATT_MAX_ENCRYPT_KEY_SIZE, &rgbLedCBs);
}

/*********************************************************************
 * @fn      RGBLED_SetParameter
 *
 * @brief   Set an RGB LED Service parameter.
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
bStatus_t RGBLED_SetParameter(uint8_t param, uint8_t len, void *value)
{
  bStatus_t ret = SUCCESS;

  switch (param)
  {
    case RGB_LED_PARAM_COLOR:
      if (len == RGB_LED_COLOR_VALUE_LEN)
      {
        if (memcmp(rgbLedColor, value, RGB_LED_COLOR_VALUE_LEN))
        {
          memcpy(rgbLedColor, value, RGB_LED_COLOR_VALUE_LEN);
          ret = GATTServApp_ProcessCharCfg(rgbLedColorClientCharCfg, rgbLedColor, FALSE,
            rgbLedAttrTbl, GATT_NUM_ATTRS(rgbLedAttrTbl),
            INVALID_TASK_ID, RGBLEDReadAttrCB);
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
 * @fn      RGBLED_GetParameter
 *
 * @brief   Get an RGB LED parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to get.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t RGBLED_GetParameter(uint8_t param, void *value)
{
  bStatus_t ret = SUCCESS;

  switch (param)
  {
    case RGB_LED_PARAM_COLOR:
      memcpy(value, rgbLedColor, RGB_LED_COLOR_VALUE_LEN);
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return (ret);
}

/*********************************************************************
 * @fn          RGBLEDReadAttrCB
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
static bStatus_t RGBLEDReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
  uint8_t *pValue, uint16_t *pLen, uint16_t offset, uint16_t maxLen,
  uint8_t method)
{
  bStatus_t status = SUCCESS;

  // Make sure it's not a blob operation (no attributes in the profile are long)
  if (offset > 0)
    return ATT_ERR_ATTR_NOT_LONG;

  uint16_t uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);

  if (uuid == RGB_LED_COLOR_UUID)
  {
    *pLen = RGB_LED_COLOR_VALUE_LEN;
    memcpy(pValue, rgbLedColor, MAX(RGB_LED_COLOR_VALUE_LEN, maxLen));
  }
  else
    status = ATT_ERR_ATTR_NOT_FOUND;

  return (status);
}

/*********************************************************************
 * @fn      RGBLEDWriteAttrCB
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
static bStatus_t RGBLEDWriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
  uint8_t *pValue, uint16_t len, uint16_t offset, uint8_t method)
{
  bStatus_t status = SUCCESS;

  // If attribute permissions require authorization to write, return error
  if (gattPermitAuthorWrite(pAttr->permissions))
  {
    // Insufficient authorization
    return ATT_ERR_INSUFFICIENT_AUTHOR;
  }

  // Make sure it's not a blob operation (no attributes in the profile are long)
  if (offset > 0)
    return ATT_ERR_ATTR_NOT_LONG;

  if (pAttr->type.len == ATT_BT_UUID_SIZE)
  {
    // 16-bit UUID
    uint16_t uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);

    switch (uuid)
    {
      case RGB_LED_COLOR_UUID:
        //Validate the value
        if (len != RGB_LED_COLOR_VALUE_LEN)
          status = ATT_ERR_INVALID_VALUE_SIZE;

        if (status == SUCCESS)
        {
          uint8 *pCurValue = (uint8 *)pAttr->pValue;

          /* Change the RGB LED color, only if the value changed */
          if (memcmp(pCurValue, pValue, RGB_LED_COLOR_VALUE_LEN))
          {
            if (rgbLedServiceColorChangeCB)
              rgbLedServiceColorChangeCB();
            memcpy(pCurValue, pValue, RGB_LED_COLOR_VALUE_LEN); // Save the value
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
  }
  else
  {
    // 128-bit UUID
    status = ATT_ERR_INVALID_HANDLE;
  }

  return status;
}

/*********************************************************************
 * @fn      RGBLED_Setup
 *
 * @brief   Set up application callback functions.
 *
 * @param   cCB - RGB LED color change callback
 *
 * @return  none.
 */
void RGBLED_Setup(rgbLedServiceColorChangeCB_t cCB)
{
  rgbLedServiceColorChangeCB = cCB;
}

/*********************************************************************
*********************************************************************/
