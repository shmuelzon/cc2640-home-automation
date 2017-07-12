#ifndef HOMEAUTOMATIONRELAY_H
#define HOMEAUTOMATIONRELAY_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "board.h"
#include "home_automation.h"

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

#if Board_RELAY_SET != PIN_UNASSIGNED
/*
 * Initialize Relay module
 */
extern void HomeAutomationRelay_init(void);

/*
 * Task Event Processor for characteristic changes
 */
void HomeAutomationRelay_processCharChangeEvt(uint8_t paramID);

/*
 * Task Event Processor for Relay module
 */
extern void HomeAutomationRelay_processEvent(void);

/*
 * Reset Relay module
 */
extern void HomeAutomationRelay_reset(void);

/*
 * Set the relay to a specific state
 */
extern void HomeAutomationRelay_StateSet(uint8_t on);

/*
 * Toggle the relay state
 */
extern void HomeAutomationRelay_Toggle(void);

#else

/* Relay module not included */
#define HomeAutomationRelay_init()
#define HomeAutomationRelay_processCharChangeEvt(paramID)
#define HomeAutomationRelay_processEvent()
#define HomeAutomationRelay_reset()
#define HomeAutomationRelay_StateSet(on)
#define HomeAutomationRelay_Toggle()

#endif // Board_RELAY_SET != PIN_UNASSIGNED

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* HOMEAUTOMATIONRELAY_H */
