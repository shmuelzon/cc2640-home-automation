/******************************************************************************

 Copyright (c) 2013-2016, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include "hci_tl.h"
#include "gatt.h"
#include "linkdb.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "battservice.h"
#include "relayservice.h"
#include "switchservice.h"

#if defined(FEATURE_OAD) || defined(IMAGE_INVALIDATE)
#include "oad_target.h"
#include "oad.h"
#endif //FEATURE_OAD || IMAGE_INVALIDATE

#include "peripheral.h"
#include "gapbondmgr.h"

#include "osal_snv.h"
#include "icall_apimsg.h"

#include "util.h"

#ifdef USE_RCOSC
#include "rcosc_calibration.h"
#endif //USE_RCOSC

#include <ti/mw/display/Display.h>
#include "board_key.h"

#include "board.h"

#include "home_automation.h"

#if defined( USE_FPGA ) || defined( DEBUG_SW_TRACE )
#include <driverlib/ioc.h>
#endif // USE_FPGA | DEBUG_SW_TRACE

/*********************************************************************
 * CONSTANTS
 */

// Advertising interval when device is discoverable (units of 625us, 1600=1s)
#define DEFAULT_ADVERTISING_INTERVAL          1600

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

#ifndef FEATURE_OAD
// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     80

// Maximum connection interval (units of 1.25ms, 200=250ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     200
#else //!FEATURE_OAD
// Minimum connection interval (units of 1.25ms, 8=10ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     8

// Maximum connection interval (units of 1.25ms, 8=10ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     8
#endif // FEATURE_OAD

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         3

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter
// update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Whether to enable automatic parameter update request when a connection is
// formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         GAPROLE_LINK_PARAM_UPDATE_INITIATE_BOTH_PARAMS

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

#ifdef FEATURE_OAD
// The size of an OAD packet.
#define OAD_PACKET_SIZE                       ((OAD_BLOCK_SIZE) + 2)
#endif // FEATURE_OAD

// Task configuration
#define HA_TASK_PRIORITY                      1

#ifndef HA_TASK_STACK_SIZE
#define HA_TASK_STACK_SIZE                    644
#endif

// Internal Events for RTOS application
#define HA_STATE_CHANGE_EVT                   (1 << 0)
#define HA_CONN_EVT_END_EVT                   (1 << 1)
#define HA_RELAY_STATE_CHANGED_EVT            (1 << 2)
#define HA_RELAY_CLEAR_GPIO_EVT               (1 << 3)
#define HA_GPIO_DEBOUNCE_EVT                  (1 << 4)
#define HA_TOGGLE_OWN_RELAY_CHANGED_EVT       (1 << 5)

// Pulse length to set/reset the relay
#define HA_RELAY_PULSE_TIME                   10

// Debounce timeout before reading GPIO state (in milliseconds)
#define HA_DEBOUNCE_TIME                      25

// Simple Non-Volatile (SNV) sections
#define HA_SNV_RELAY_STATE                    (BLE_NVID_CUST_START + 0)
#define HA_SNV_TOGGLE_OWN_RELAY               (BLE_NVID_CUST_START + 1)

/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr;  // event header.
} haEvt_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Display Interface
Display_Handle dispHandle = NULL;

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Semaphore globally used to post events to the application thread
static ICall_Semaphore sem;

// Clock instances
static Clock_Struct periodRelay;
static Clock_Struct periodDebounce;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

#if defined(FEATURE_OAD)
// Event data from OAD profile.
static Queue_Struct oadQ;
static Queue_Handle hOadQ;
#endif //FEATURE_OAD

// events flag for internal application events.
static uint16_t events;

// Task configuration
Task_Struct haTask;
Char haTaskStack[HA_TASK_STACK_SIZE];

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8_t scanRspData[] =
{
  // complete name
  12,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'S',
  'w',
  'i',
  't',
  'c',
  'h',
  '-',
  'X',
  'X',
  'X',
  'X',

  // connection interval range
  5,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),   // 100ms
  HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
  LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),   // 250ms
  HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),

  // Tx power level
  2,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0    // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertising)
static uint8_t advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  2,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
#if defined(FEATURE_OAD)
  3,   // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
  LO_UINT16(OAD_SERVICE_UUID),
  HI_UINT16(OAD_SERVICE_UUID),
#endif //FEATURE_OAD
};

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Switch-XXXX";

// Globals used for ATT Response retransmission
static gattMsgEvent_t *pAttRsp = NULL;
static uint8_t rspTxRetry = 0;

// GPIOs
static PIN_State haPinState;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void HomeAutomation_init( void );
static void HomeAutomation_taskFxn(UArg a0, UArg a1);

static uint8_t HomeAutomation_processStackMsg(ICall_Hdr *pMsg);
static uint8_t HomeAutomation_processGATTMsg(gattMsgEvent_t *pMsg);
static void HomeAutomation_processAppMsg(haEvt_t *pMsg);
static void HomeAutomation_processStateChangeEvt(gaprole_States_t newState);

static void HomeAutomation_sendAttRsp(void);
static void HomeAutomation_freeAttRsp(uint8_t status);

static void HomeAutomation_stateChangeCB(gaprole_States_t newState);
static void HomeAutomation_enqueueMsg(uint8_t event, uint8_t state);

static void HomeAutomation_clockCb(UArg arg);
static void HomeAutomation_intCb(PIN_Handle handle, PIN_Id pinId);

#ifdef FEATURE_OAD
void HomeAutomation_processOadWriteCB(uint8_t event, uint16_t connHandle,
                                           uint8_t *pData);
#endif //FEATURE_OAD

static void HomeAutomation_RelayStateSet(uint8_t on);
static void HomeAutomation_RelayStateStop(void);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t HomeAutomation_gapRoleCBs =
{
  HomeAutomation_stateChangeCB     // Profile State Change Callbacks
};

// GAP Bond Manager Callbacks
static gapBondCBs_t HomeAutomation_BondMgrCBs =
{
  NULL, // Passcode callback (not used by application)
  NULL  // Pairing / Bonding state Callback (not used by application)
};

#ifdef FEATURE_OAD
static oadTargetCBs_t HomeAutomation_oadCBs =
{
  HomeAutomation_processOadWriteCB // Write Callback.
};
#endif //FEATURE_OAD

#if Board_RELAY_SET != PIN_UNASSIGNED
static void HomeAutomation_RelayStateChangeCB(void);
#endif
#if Board_SWITCH != PIN_UNASSIGNED
static void HomeAutomation_ToggleOwnRelayChangeCB(void);
#endif

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      HomeAutomation_createTask
 *
 * @brief   Task creation function for the Home Automation task.
 *
 * @param   None.
 *
 * @return  None.
 */
void HomeAutomation_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = haTaskStack;
  taskParams.stackSize = HA_TASK_STACK_SIZE;
  taskParams.priority = HA_TASK_PRIORITY;

  Task_construct(&haTask, HomeAutomation_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      HomeAutomation_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (i.e. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 *
 * @param   None.
 *
 * @return  None.
 */
static void HomeAutomation_init(void)
{
  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &sem);

#ifdef USE_RCOSC
  RCOSC_enableCalibration();
#endif // USE_RCOSC

#if defined( USE_FPGA )
  // configure RF Core SMI Data Link
  IOCPortConfigureSet(IOID_12, IOC_PORT_RFC_GPO0, IOC_STD_OUTPUT);
  IOCPortConfigureSet(IOID_11, IOC_PORT_RFC_GPI0, IOC_STD_INPUT);

  // configure RF Core SMI Command Link
  IOCPortConfigureSet(IOID_10, IOC_IOCFG0_PORT_ID_RFC_SMI_CL_OUT, IOC_STD_OUTPUT);
  IOCPortConfigureSet(IOID_9, IOC_IOCFG0_PORT_ID_RFC_SMI_CL_IN, IOC_STD_INPUT);

  // configure RF Core tracer IO
  IOCPortConfigureSet(IOID_8, IOC_PORT_RFC_TRC, IOC_STD_OUTPUT);
#else // !USE_FPGA
  #if defined( DEBUG_SW_TRACE )
    // configure RF Core tracer IO
    IOCPortConfigureSet(IOID_8, IOC_PORT_RFC_TRC, IOC_STD_OUTPUT | IOC_CURRENT_4MA | IOC_SLEW_ENABLE);
  #endif // DEBUG_SW_TRACE
#endif // USE_FPGA

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);

  // Create clocks
  Util_constructClock(&periodRelay, HomeAutomation_clockCb,
    HA_RELAY_PULSE_TIME, 0, false, HA_RELAY_CLEAR_GPIO_EVT);
  Util_constructClock(&periodDebounce, HomeAutomation_clockCb,
    HA_DEBOUNCE_TIME, 0, false, HA_GPIO_DEBOUNCE_EVT);

  // Setup GPIO
  PIN_open(&haPinState, BoardGpioInitTable);
  PIN_registerIntCb(&haPinState, HomeAutomation_intCb);

  dispHandle = Display_open(Display_Type_LCD, NULL);

  // Setup the GAP
  GAP_SetParamValue(TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL);

  // Setup the GAP Peripheral Role Profile
  {
    // For all hardware platforms, device starts advertising upon initialization
    uint8_t initialAdvertEnable = TRUE;

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16_t advertOffTime = 0;

    uint8_t enableUpdateRequest = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16_t desiredMinInterval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16_t desiredMaxInterval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16_t desiredSlaveLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16_t desiredConnTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                         &initialAdvertEnable);
    GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                         &advertOffTime);

    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);

    GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE, sizeof(uint8_t),
                         &enableUpdateRequest);
    GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMinInterval);
    GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMaxInterval);
    GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16_t),
                         &desiredSlaveLatency);
    GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16_t),
                         &desiredConnTimeout);
  }

  // Set advertising interval
  {
    uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);
  }

  // Setup the GAP Bond Manager
  {
    uint32_t passkey = 0; // passkey "000000"
    uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8_t mitm = TRUE;
    uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8_t bonding = TRUE;

    GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32_t),
                            &passkey);
    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
  }

   // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);           // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes
  DevInfo_AddService();                        // Device Information Service
  Batt_AddService();                           // Battery Service
  Batt_Setup(2100, 3000, NULL, NULL);

#ifdef FEATURE_OAD
  VOID OAD_addService();                 // OAD Profile
  OAD_register((oadTargetCBs_t *)&HomeAutomation_oadCBs);
  hOadQ = Util_constructQueue(&oadQ);
#endif //FEATURE_OAD

#ifdef IMAGE_INVALIDATE
  Reset_addService();
#endif //IMAGE_INVALIDATE

  // Relay service
#if Board_RELAY_SET != PIN_UNASSIGNED
  {
    uint8_t relayState;

    Relay_AddService();
    Relay_Setup(HomeAutomation_RelayStateChangeCB);
    // Load last state from persistent storage
    if (osal_snv_read(HA_SNV_RELAY_STATE, sizeof(relayState), &relayState) == NV_OPER_FAILED)
       relayState = 0xFF;
    Relay_SetParameter(RELAY_PARAM_STATE, sizeof(relayState), &relayState);
  }
#endif

  // Switch service
#if Board_SWITCH != PIN_UNASSIGNED
  {
    uint8_t switchState, toggleOwnRelay;

    Switch_AddService();
    Switch_Setup(HomeAutomation_ToggleOwnRelayChangeCB);
    // Set up interrupts on switch
    PIN_setInterrupt(&haPinState, Board_SWITCH | PIN_IRQ_BOTHEDGES);
    // Set current state of switch
    switchState = PIN_getInputValue(Board_SWITCH);
    Switch_SetParameter(SWITCH_PARAM_STATE, 1, &switchState);
    // Load configuration from persistent storage
    if (osal_snv_read(HA_SNV_TOGGLE_OWN_RELAY, sizeof(toggleOwnRelay), &toggleOwnRelay) == NV_OPER_FAILED)
      toggleOwnRelay = 1;
    Switch_SetParameter(SWITCH_PARAM_TOGGLE_OWN_RELAY, sizeof(toggleOwnRelay), &toggleOwnRelay);
  }
#endif

  // Start the Device
  VOID GAPRole_StartDevice(&HomeAutomation_gapRoleCBs);

  // Start Bond Manager
  VOID GAPBondMgr_Register(&HomeAutomation_BondMgrCBs);

  // Register with GAP for HCI/Host messages
  GAP_RegisterForMsgs(selfEntity);

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);

  HCI_LE_ReadMaxDataLenCmd();

#if defined FEATURE_OAD
#if defined (HAL_IMAGE_A)
  Display_print0(dispHandle, 0, 0, "Switch A");
#else
  Display_print0(dispHandle, 0, 0, "Switch B");
#endif // HAL_IMAGE_A
#else
  Display_print0(dispHandle, 0, 0, "Switch");
#endif // FEATURE_OAD
}

/*********************************************************************
 * @fn      HomeAutomation_taskFxn
 *
 * @brief   Application task entry point for the Home Automation task.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
static void HomeAutomation_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  HomeAutomation_init();

  // Application main loop
  for (;;)
  {
    // Waits for a signal to the semaphore associated with the calling thread.
    // Note that the semaphore associated with a thread is signaled when a
    // message is queued to the message receive queue of the thread or when
    // ICall_signal() function is called onto the semaphore.
    ICall_Errno errno = ICall_wait(ICALL_TIMEOUT_FOREVER);

    if (errno == ICALL_ERRNO_SUCCESS)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        uint8 safeToDealloc = TRUE;

        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;

          // Check for BLE stack events first
          if (pEvt->signature == 0xffff)
          {
            if (pEvt->event_flag & HA_CONN_EVT_END_EVT)
            {
              // Try to retransmit pending ATT Response (if any)
              HomeAutomation_sendAttRsp();
            }
          }
          else
          {
            // Process inter-task message
            safeToDealloc = HomeAutomation_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message.
      while (!Queue_empty(appMsgQueue))
      {
        haEvt_t *pMsg = (haEvt_t *)Util_dequeueMsg(appMsgQueue);
        if (pMsg)
        {
          // Process message.
          HomeAutomation_processAppMsg(pMsg);

          // Free the space from the message.
          ICall_free(pMsg);
        }
      }
    }

#ifdef FEATURE_OAD
    while (!Queue_empty(hOadQ))
    {
      oadTargetWrite_t *oadWriteEvt = Queue_get(hOadQ);

      // Identify new image.
      if (oadWriteEvt->event == OAD_WRITE_IDENTIFY_REQ)
      {
        OAD_imgIdentifyWrite(oadWriteEvt->connHandle, oadWriteEvt->pData);
      }
      // Write a next block request.
      else if (oadWriteEvt->event == OAD_WRITE_BLOCK_REQ)
      {
        OAD_imgBlockWrite(oadWriteEvt->connHandle, oadWriteEvt->pData);
      }

      // Free buffer.
      ICall_free(oadWriteEvt);
    }
#endif //FEATURE_OAD

    if (events & HA_RELAY_STATE_CHANGED_EVT)
    {
      uint8_t state;

      events &= ~HA_RELAY_STATE_CHANGED_EVT;

      Relay_GetParameter(RELAY_PARAM_STATE, &state);
      HomeAutomation_RelayStateSet(state);
    }

    if (events & HA_RELAY_CLEAR_GPIO_EVT)
    {
      events &= ~HA_RELAY_CLEAR_GPIO_EVT;

      HomeAutomation_RelayStateStop();
    }

    if (events & HA_GPIO_DEBOUNCE_EVT)
    {
      events &= ~HA_GPIO_DEBOUNCE_EVT;

#if Board_SWITCH != PIN_UNASSIGNED
      uint8_t prev, cur;

      Switch_GetParameter(SWITCH_PARAM_STATE, &prev);
      cur = PIN_getInputValue(Board_SWITCH);

      if (prev != cur)
      {
        uint8_t doToggle = 0;

        // Save new switch state
        Switch_SetParameter(SWITCH_PARAM_STATE, 1, &cur);

#if Board_RELAY_SET != PIN_UNASSIGNED
        // Check if we need to toggle the relay
        Switch_GetParameter(SWITCH_PARAM_TOGGLE_OWN_RELAY, &doToggle);
#endif

        if (doToggle)
        {
          uint8_t relayState;

          Relay_GetParameter(RELAY_PARAM_STATE, &relayState);
          relayState = !relayState;
          Relay_SetParameter(RELAY_PARAM_STATE, sizeof(relayState), &relayState);
          HomeAutomation_RelayStateSet(relayState);
        }
      }
#endif
    }

    if (events & HA_TOGGLE_OWN_RELAY_CHANGED_EVT)
    {
      uint8_t setting;

      events &= ~HA_TOGGLE_OWN_RELAY_CHANGED_EVT;

      Switch_GetParameter(SWITCH_PARAM_TOGGLE_OWN_RELAY, &setting);
      osal_snv_write(HA_SNV_TOGGLE_OWN_RELAY, sizeof(setting), &setting);
    }
  }
}

/*********************************************************************
 * @fn      HomeAutomation_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t HomeAutomation_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;

  switch (pMsg->event)
  {
    case GATT_MSG_EVENT:
      // Process GATT message
      safeToDealloc = HomeAutomation_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
      {
        // Process HCI message
        switch(pMsg->status)
        {
          case HCI_COMMAND_COMPLETE_EVENT_CODE:
            // Process HCI Command Complete Event
            break;

          default:
            break;
        }
      }
      break;

    default:
      // do nothing
      break;
  }

  return (safeToDealloc);
}

/*********************************************************************
 * @fn      HomeAutomation_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t HomeAutomation_processGATTMsg(gattMsgEvent_t *pMsg)
{
  // See if GATT server was unable to transmit an ATT response
  if (pMsg->hdr.status == blePending)
  {
    // No HCI buffer was available. Let's try to retransmit the response
    // on the next connection event.
    if (HCI_EXT_ConnEventNoticeCmd(pMsg->connHandle, selfEntity,
                                   HA_CONN_EVT_END_EVT) == SUCCESS)
    {
      // First free any pending response
      HomeAutomation_freeAttRsp(FAILURE);

      // Hold on to the response message for retransmission
      pAttRsp = pMsg;

      // Don't free the response message yet
      return (FALSE);
    }
  }
  else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
  {
    // ATT request-response or indication-confirmation flow control is
    // violated. All subsequent ATT requests or indications will be dropped.
    // The app is informed in case it wants to drop the connection.

    // Display the opcode of the message that caused the violation.
    Display_print1(dispHandle, 5, 0, "FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
  }
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
    // MTU size updated
    Display_print1(dispHandle, 5, 0, "MTU Size: $d", pMsg->msg.mtuEvt.MTU);
  }

  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);

  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
 * @fn      HomeAutomation_sendAttRsp
 *
 * @brief   Send a pending ATT response message.
 *
 * @param   none
 *
 * @return  none
 */
static void HomeAutomation_sendAttRsp(void)
{
  // See if there's a pending ATT Response to be transmitted
  if (pAttRsp != NULL)
  {
    uint8_t status;

    // Increment retransmission count
    rspTxRetry++;

    // Try to retransmit ATT response till either we're successful or
    // the ATT Client times out (after 30s) and drops the connection.
    status = GATT_SendRsp(pAttRsp->connHandle, pAttRsp->method, &(pAttRsp->msg));
    if ((status != blePending) && (status != MSG_BUFFER_NOT_AVAIL))
    {
      // Disable connection event end notice
      HCI_EXT_ConnEventNoticeCmd(pAttRsp->connHandle, selfEntity, 0);

      // We're done with the response message
      HomeAutomation_freeAttRsp(status);
    }
    else
    {
      // Continue retrying
      Display_print1(dispHandle, 5, 0, "Rsp send retry: %d", rspTxRetry);
    }
  }
}

/*********************************************************************
 * @fn      HomeAutomation_freeAttRsp
 *
 * @brief   Free ATT response message.
 *
 * @param   status - response transmit status
 *
 * @return  none
 */
static void HomeAutomation_freeAttRsp(uint8_t status)
{
  // See if there's a pending ATT response message
  if (pAttRsp != NULL)
  {
    // See if the response was sent out successfully
    if (status == SUCCESS)
    {
      Display_print1(dispHandle, 5, 0, "Rsp sent retry: %d", rspTxRetry);
    }
    else
    {
      // Free response payload
      GATT_bm_free(&pAttRsp->msg, pAttRsp->method);

      Display_print1(dispHandle, 5, 0, "Rsp retry failed: %d", rspTxRetry);
    }

    // Free response message
    ICall_freeMsg(pAttRsp);

    // Reset our globals
    pAttRsp = NULL;
    rspTxRetry = 0;
  }
}

/*********************************************************************
 * @fn      HomeAutomation_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void HomeAutomation_processAppMsg(haEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
    case HA_STATE_CHANGE_EVT:
      HomeAutomation_processStateChangeEvt((gaprole_States_t)pMsg->
                                                hdr.state);
      break;

    default:
      // Do nothing.
      break;
  }
}

/*********************************************************************
 * @fn      HomeAutomation_stateChangeCB
 *
 * @brief   Callback from GAP Role indicating a role state change.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void HomeAutomation_stateChangeCB(gaprole_States_t newState)
{
  HomeAutomation_enqueueMsg(HA_STATE_CHANGE_EVT, newState);
}

/*********************************************************************
 * @fn      HomeAutomation_processStateChangeEvt
 *
 * @brief   Process a pending GAP Role state change event.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void HomeAutomation_processStateChangeEvt(gaprole_States_t newState)
{
#ifdef PLUS_BROADCASTER
  static bool firstConnFlag = false;
#endif // PLUS_BROADCASTER

  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8_t ownAddress[B_ADDR_LEN];
        uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];
        char mac_str[4];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

        // Add MAC address to device name
#define TO_HEX(i) ("0123456789ABCDEF"[i])
        mac_str[0] = TO_HEX((ownAddress[1] >> 4) & 0x0F);
        mac_str[1] = TO_HEX(ownAddress[1] & 0x0F);
        mac_str[2] = TO_HEX((ownAddress[0] >> 4) & 0x0F);
        mac_str[3] = TO_HEX(ownAddress[0] & 0x0F);
        memcpy(&scanRspData[9], mac_str, 4);
        memcpy(&attDeviceName[7], mac_str, 4);

        GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData),
                             scanRspData);
        GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);
#undef TO_HEX

        // Display device address
        Display_print0(dispHandle, 1, 0, Util_convertBdAddr2Str(ownAddress));
        Display_print0(dispHandle, 2, 0, "Initialized");
      }
      break;

    case GAPROLE_ADVERTISING:
      Display_print0(dispHandle, 2, 0, "Advertising");
      break;

#ifdef PLUS_BROADCASTER
    /* After a connection is dropped a device in PLUS_BROADCASTER will continue
     * sending non-connectable advertisements and shall sending this change of
     * state to the application.  These are then disabled here so that sending
     * connectable advertisements can resume.
     */
    case GAPROLE_ADVERTISING_NONCONN:
      {
        uint8_t advertEnabled = FALSE;

        // Disable non-connectable advertising.
        GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                           &advertEnabled);

        advertEnabled = TRUE;

        // Enabled connectable advertising.
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                             &advertEnabled);

        // Reset flag for next connection.
        firstConnFlag = false;

        HomeAutomation_freeAttRsp(bleNotConnected);
      }
      break;
#endif //PLUS_BROADCASTER

    case GAPROLE_CONNECTED:
      {
        linkDBInfo_t linkInfo;
        uint8_t numActive = 0;

        numActive = linkDB_NumActive();

        // Use numActive to determine the connection handle of the last
        // connection
        if ( linkDB_GetInfo( numActive - 1, &linkInfo ) == SUCCESS )
        {
          Display_print1(dispHandle, 2, 0, "Num Conns: %d", (uint16_t)numActive);
          Display_print0(dispHandle, 3, 0, Util_convertBdAddr2Str(linkInfo.addr));
        }
        else
        {
          uint8_t peerAddress[B_ADDR_LEN];

          GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, peerAddress);

          Display_print0(dispHandle, 2, 0, "Connected");
          Display_print0(dispHandle, 3, 0, Util_convertBdAddr2Str(peerAddress));
        }

        #ifdef PLUS_BROADCASTER
          // Only turn advertising on for this state when we first connect
          // otherwise, when we go from connected_advertising back to this state
          // we will be turning advertising back on.
          if (firstConnFlag == false)
          {
            uint8_t advertEnabled = FALSE; // Turn on Advertising

            // Disable connectable advertising.
            GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                                 &advertEnabled);

            // Set to true for non-connectable advertising.
            advertEnabled = TRUE;

            // Enable non-connectable advertising.
            GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                                 &advertEnabled);
            firstConnFlag = true;
          }
        #endif // PLUS_BROADCASTER
      }
      break;

    case GAPROLE_CONNECTED_ADV:
      Display_print0(dispHandle, 2, 0, "Connected Advertising");
      break;

    case GAPROLE_WAITING:
      HomeAutomation_freeAttRsp(bleNotConnected);

      Display_print0(dispHandle, 2, 0, "Disconnected");

      // Clear remaining lines
      Display_clearLines(dispHandle, 3, 5);
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      HomeAutomation_freeAttRsp(bleNotConnected);

      Display_print0(dispHandle, 2, 0, "Timed Out");

      // Clear remaining lines
      Display_clearLines(dispHandle, 3, 5);

      #ifdef PLUS_BROADCASTER
        // Reset flag for next connection.
        firstConnFlag = false;
      #endif //PLUS_BROADCASTER
      break;

    case GAPROLE_ERROR:
      Display_print0(dispHandle, 2, 0, "Error");
      break;

    default:
      Display_clearLine(dispHandle, 2);
      break;
  }
}

#ifdef FEATURE_OAD
/*********************************************************************
 * @fn      HomeAutomation_processOadWriteCB
 *
 * @brief   Process a write request to the OAD profile.
 *
 * @param   event      - event type:
 *                       OAD_WRITE_IDENTIFY_REQ
 *                       OAD_WRITE_BLOCK_REQ
 * @param   connHandle - the connection Handle this request is from.
 * @param   pData      - pointer to data for processing and/or storing.
 *
 * @return  None.
 */
void HomeAutomation_processOadWriteCB(uint8_t event, uint16_t connHandle,
                                           uint8_t *pData)
{
  oadTargetWrite_t *oadWriteEvt = ICall_malloc( sizeof(oadTargetWrite_t) + \
                                             sizeof(uint8_t) * OAD_PACKET_SIZE);

  if ( oadWriteEvt != NULL )
  {
    oadWriteEvt->event = event;
    oadWriteEvt->connHandle = connHandle;

    oadWriteEvt->pData = (uint8_t *)(&oadWriteEvt->pData + 1);
    memcpy(oadWriteEvt->pData, pData, OAD_PACKET_SIZE);

    Queue_put(hOadQ, (Queue_Elem *)oadWriteEvt);

    // Post the application's semaphore.
    Semaphore_post(sem);
  }
  else
  {
    // Fail silently.
  }
}
#endif //FEATURE_OAD

/*********************************************************************
 * @fn      HomeAutomation_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 *
 * @return  None.
 */
static void HomeAutomation_enqueueMsg(uint8_t event, uint8_t state)
{
  haEvt_t *pMsg;

  // Create dynamic pointer to message.
  if ((pMsg = ICall_malloc(sizeof(haEvt_t))))
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;

    // Enqueue the message.
    Util_enqueueMsg(appMsgQueue, sem, (uint8*)pMsg);
  }
}

/*********************************************************************
 * @fn      HomeAutomation_clockCb
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void HomeAutomation_clockCb(UArg arg)
{
  // Store the event.
  events |= arg;

  // Wake up the application.
  Semaphore_post(sem);
}

#if Board_RELAY_SET != PIN_UNASSIGNED
/*********************************************************************
 * @fn      HomeAutomation_RelayStateChangeCB
 *
 * @brief   Callback function to be called when relay state is changed.
 *
 * @return  None.
 */
static void HomeAutomation_RelayStateChangeCB(void)
{
  // Store the event.
  events |= HA_RELAY_STATE_CHANGED_EVT;

  // Wake up the application.
  Semaphore_post(sem);
}
#endif

/*********************************************************************
 * @fn      HomeAutomation_RelayStateSet
 *
 * @brief   Start setting the relay to the defined state.
 *
 * @param   on - the state to set the relay.
 *
 * @return  None.
 */
static void HomeAutomation_RelayStateSet(uint8_t on)
{
  if (Util_isActive(&periodRelay))
    Util_stopClock(&periodRelay);

  // Reset both GPIOs
  PIN_setOutputValue(&haPinState, Board_RELAY_SET, 0);
  PIN_setOutputValue(&haPinState, Board_RELAY_RESET, 0);

  // Set only relevant GPIO
  PIN_setOutputValue(&haPinState, on ? Board_RELAY_SET : Board_RELAY_RESET, 1);

  // Start a timer to reset the GPIOs
  Util_startClock(&periodRelay);

  // Save new state in persistent storage
  osal_snv_write(HA_SNV_RELAY_STATE, sizeof(on), &on);
}

/*********************************************************************
 * @fn      HomeAutomation_RelayStateStop
 *
 * @brief   Finish setting the relay state.
 *
 * @return  None.
 */
static void HomeAutomation_RelayStateStop(void)
{
  // Reset both GPIOs
  PIN_setOutputValue(&haPinState, Board_RELAY_SET, 0);
  PIN_setOutputValue(&haPinState, Board_RELAY_RESET, 0);
}

/*********************************************************************
 * @fn      HomeAutomation_intCb
 *
 * @brief   Callback function to be called on GPIO interrupt.
 *
 * @param   handle - Pin driver handle.
 * @param   pinId - Pin ID.
 *
 * @return  None.
 */
static void HomeAutomation_intCb(PIN_Handle handle, PIN_Id pinId)
{
  // Interrupt was triggered, start debouce timer
  if (pinId == Board_SWITCH)
    Util_startClock(&periodDebounce);
}

#if Board_SWITCH != PIN_UNASSIGNED
/*********************************************************************
 * @fn      HomeAutomation_ToggleOwnRelayChangeCB
 *
 * @brief   Callback function to be called when the toggleOwnRelay
 *          setting is changed.
 *
 * @return  None.
 */
static void HomeAutomation_ToggleOwnRelayChangeCB(void)
{
  // Store the event.
  events |= HA_TOGGLE_OWN_RELAY_CHANGED_EVT;

  // Wake up the application.
  Semaphore_post(sem);
}
#endif

/*********************************************************************
*********************************************************************/
