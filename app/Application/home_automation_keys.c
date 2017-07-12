/*********************************************************************
 * INCLUDES
 */
#include "gatt.h"
#include "gattservapp.h"
#include "home_automation_keys.h"
#include "home_automation_relay.h"
#include "board.h"
#include "peripheral.h"
#include "util.h"
#include "switchservice.h"
#include "contactsensorservice.h"

#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>

#ifdef HAS_KEYS
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
// Debounce timeout before reading GPIO state (in milliseconds)
#define KEYS_DEBOUNCE_TIME       25

// Events
#define KEYS_DEBOUNCE_EVT        (1 << 0)

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
static Clock_Struct periodDebounce;
static uint8_t events;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void HomeAutomationKeys_clockHandler(UArg arg);

/*********************************************************************
 * PROFILE CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      HomeAutomationKeys_init
 *
 * @brief   Initialization function for the HomeAutomation keys
 *
 * @param   none
 *
 * @return  none
 */
void HomeAutomationKeys_init(void)
{
#ifdef HAS_SWITCH
  // Add switch service
  Switch_AddService();
#endif

#ifdef HAS_CONTACT
  // Add contact sensor service
  ContactSensor_AddService();
#endif

  // Initialize the module state variables
  HomeAutomationKeys_reset();

  // Create one-shot clock for key press debouncing
  Util_constructClock(&periodDebounce, HomeAutomationKeys_clockHandler,
    KEYS_DEBOUNCE_TIME, 0, false, KEYS_DEBOUNCE_EVT);
}

/*********************************************************************
 * @fn      HomeAutomationKeys_processKeySwitch
 *
 * @brief   Interrupt handler for switch
 *
 * @param   none
 *
 * @return  none
 */
#ifdef HAS_SWITCH
void HomeAutomationKeys_processKeySwitch(void)
{
  if (!Util_isActive(&periodDebounce))
    Util_startClock(&periodDebounce);
}
#endif

/*********************************************************************
 * @fn      HomeAutomationKeys_processKeyContact
 *
 * @brief   Interrupt handler for contact sensor (reed switch)
 *
 * @param   none
 *
 * @return  none
 */
#ifdef HAS_CONTACT
void HomeAutomationKeys_processKeyContact(void)
{
  if (!Util_isActive(&periodDebounce))
    Util_startClock(&periodDebounce);
}
#endif

/*********************************************************************
 * @fn      HomeAutomationKeys_processEvent
 *
 * @brief   HomeAutomation Keys event processor.
 *
 * @param   none
 *
 * @return  none
 */
void HomeAutomationKeys_processEvent(void)
{
  if (events & KEYS_DEBOUNCE_EVT)
  {
    uint8_t prev, cur;

    events &= ~KEYS_DEBOUNCE_EVT;

#ifdef HAS_SWITCH
    Switch_GetParameter(SWITCH_PARAM_STATE, &prev);
    cur = PIN_getInputValue(Board_SWITCH);

    if (prev != cur)
    {
      // Save new switch state
      Switch_SetParameter(SWITCH_PARAM_STATE, 1, &cur);
      HomeAutomationRelay_Toggle();
    }
#endif

#ifdef HAS_CONTACT
    ContactSensor_GetParameter(CONTACT_SENSOR_PARAM_STATE, &prev);
    cur = PIN_getInputValue(Board_CONTACT);

    // Save new contact sensor state
    if (prev != cur)
      ContactSensor_SetParameter(CONTACT_SENSOR_PARAM_STATE, 1, &cur);
#endif
  }
}

/*********************************************************************
 * @fn      HomeAutomationKeys_reset
 *
 * @brief   Reset key state
 *
 * @param   none
 *
 * @return  none
 */
void HomeAutomationKeys_reset(void)
{
#ifdef HAS_SWITCH
  {
    uint8_t switchState;

    // Set current state of switch
    switchState = PIN_getInputValue(Board_SWITCH);
    Switch_SetParameter(SWITCH_PARAM_STATE, 1, &switchState);
  }
#endif
#ifdef HAS_CONTACT
  {
    uint8_t contactSensorState;

    // Set current state of contact sensor
    contactSensorState = PIN_getInputValue(Board_CONTACT);
    ContactSensor_SetParameter(CONTACT_SENSOR_PARAM_STATE, 1, &contactSensorState);
  }
#endif
  events = 0;
}

/*********************************************************************
 * @fn      HomeAutomationKeys_clockHandler
 *
 * @brief   Handler function for clock time-outs.
 *
 * @param   arg - event type
 *
 * @return  none
 */
static void HomeAutomationKeys_clockHandler(UArg arg)
{
  // Store the event.
  events |= arg;

  // Wake up the application.
  Semaphore_post(sem);
}

#endif // HAS_KEYS

/*********************************************************************
*********************************************************************/
