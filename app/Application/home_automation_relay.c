/*********************************************************************
 * INCLUDES
 */
#include "osal_snv.h"
#include "gatt.h"
#include "gattservapp.h"
#include "home_automation_relay.h"
#include "home_automation_rgb_led.h"
#include "board.h"
#include "peripheral.h"
#include "util.h"
#include "relayservice.h"

#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>

#if Board_RELAY_SET != PIN_UNASSIGNED

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
// Pulse length to set/reset the relay
#define RELAY_PULSE_TIME                   10

// Simple Non-Volatile (SNV) sections
#define RELAY_SNV_STATE                    (BLE_NVID_CUST_START + 0)

// Events
#define RELAY_STATE_CHANGED_EVT            (1 << 0)
#define RELAY_CLEAR_GPIO_EVT               (1 << 1)

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
// Clock instances
static Clock_Struct periodRelay;
static uint8_t events;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void HomeAutomationRelay_clockHandler(UArg arg);
static void HomeAutomationRelay_StateStop(void);
static void HomeAutomationRelay_StateChangeCB(void);

/*********************************************************************
 * PROFILE CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      HomeAutomationRelay_init
 *
 * @brief   Initialization function for the SensorTag keys
 *
 * @param   none
 *
 * @return  none
 */
void HomeAutomationRelay_init(void)
{
  // Add relay service
  Relay_AddService();
  Relay_Setup(HomeAutomationRelay_StateChangeCB);

  // Initialize the module state variables
  HomeAutomationRelay_reset();

  // Create one-shot clock for releasing relay
  Util_constructClock(&periodRelay, HomeAutomationRelay_clockHandler,
    RELAY_PULSE_TIME, 0, false, RELAY_CLEAR_GPIO_EVT);
}

/*********************************************************************
 * @fn      HomeAutomationRelay_processCharChangeEvt
 *
 * @brief   HomeAutomation relay monitor event handling
 *
 * @param   event - event identifier
 *
 */
void HomeAutomationRelay_processCharChangeEvt(uint8_t event)
{
  if (event == RELAY_STATE_CHANGED_EVT)
  {
      uint8_t state;

      Relay_GetParameter(RELAY_PARAM_STATE, &state);
      HomeAutomationRelay_StateSet(state);
  }
}

/*********************************************************************
 * @fn      HomeAutomationRelay_processEvent
 *
 * @brief   Relay event processor.
 *
 * @param   none
 *
 * @return  none
 */
void HomeAutomationRelay_processEvent(void)
{
  if (events & RELAY_CLEAR_GPIO_EVT)
  {
    events &= ~RELAY_CLEAR_GPIO_EVT;

    HomeAutomationRelay_StateStop();
  }
}

/*********************************************************************
 * @fn      HomeAutomationRelay_reset
 *
 * @brief   Reset relay state value from NVRAM
 *
 * @param   none
 *
 * @return  none
 */
void HomeAutomationRelay_reset(void)
{
  uint8_t relayState;

#if Board_RELAY_RESET != PIN_UNASSIGNED
  // Load last state from persistent storage
  if (osal_snv_read(RELAY_SNV_STATE, sizeof(relayState), &relayState) == NV_OPER_FAILED)
     relayState = 0xFF;
#else
  relayState = PIN_getOutputValue(Board_RELAY_SET);
#endif
  Relay_SetParameter(RELAY_PARAM_STATE, sizeof(relayState), &relayState);

  events = 0;
}

/*********************************************************************
 * @fn      HomeAutomationRelay_clockHandler
 *
 * @brief   Handler function for clock time-outs.
 *
 * @param   arg - event type
 *
 * @return  none
 */
static void HomeAutomationRelay_clockHandler(UArg arg)
{
  // Store the event.
  events |= arg;

  // Wake up the application.
  Semaphore_post(sem);
}

/*********************************************************************
 * @fn      HomeAutomationRelay_StateChangeCB
 *
 * @brief   Callback function to be called when relay state is changed.
 *
 * @return  None.
 */
static void HomeAutomationRelay_StateChangeCB(void)
{
  // Wake up the application thread
  HomeAutomation_charValueChangeCB(SERVICE_ID_RELAY, RELAY_STATE_CHANGED_EVT);
}

/*********************************************************************
 * @fn      HomeAutomationRelay_StateSet
 *
 * @brief   Start setting the relay to the defined state.
 *
 * @param   on - the state to set the relay.
 *
 * @return  None.
 */
void HomeAutomationRelay_StateSet(uint8_t on)
{
  int i;

  if (Util_isActive(&periodRelay))
    Util_stopClock(&periodRelay);

#if Board_RELAY_RESET != PIN_UNASSIGNED
  // Reset both GPIOs
  PIN_setOutputValue(haPinHandle, Board_RELAY_SET, 0);
  PIN_setOutputValue(haPinHandle, Board_RELAY_RESET, 0);

  // Set only relevant GPIO
  PIN_setOutputValue(haPinHandle, on ? Board_RELAY_SET : Board_RELAY_RESET, 1);

  // Start a timer to reset the GPIOs
  Util_startClock(&periodRelay);
#else
  PIN_setOutputValue(haPinHandle, Board_RELAY_SET, on);
#endif

  for (i = 0; i < Board_RGB_NUM_LEDS; i++)
    HomeAutomationRGBLED_SetLedColor(i, on ? 50 : 0, 0, 0);
  HomeAutomationRGBLED_Update();

  // Save new state in persistent storage
  osal_snv_write(RELAY_SNV_STATE, sizeof(on), &on);
}

/*********************************************************************
 * @fn      HomeAutomationRelay_StateStop
 *
 * @brief   Finish setting the relay state, relevant only for latching relays.
 *
 * @return  None.
 */
static void HomeAutomationRelay_StateStop(void)
{
  // Reset both GPIOs
  PIN_setOutputValue(haPinHandle, Board_RELAY_SET, 0);
  PIN_setOutputValue(haPinHandle, Board_RELAY_RESET, 0);
}

/*********************************************************************
 * @fn      HomeAutomationRelay_Toggle
 *
 * @brief   Toggle the relay state.
 *
 * @return  None.
 */
void HomeAutomationRelay_Toggle(void)
{
  uint8_t relayState;

  Relay_GetParameter(RELAY_PARAM_STATE, &relayState);
  relayState = !relayState;
  Relay_SetParameter(RELAY_PARAM_STATE, sizeof(relayState), &relayState);
  HomeAutomationRelay_StateSet(relayState);
}

#endif // Board_RELAY_SET != PIN_UNASSIGNED

/*********************************************************************
*********************************************************************/
