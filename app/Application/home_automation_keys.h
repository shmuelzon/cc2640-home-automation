#ifndef HOMEAUTOMATIONKEYS_H
#define HOMEAUTOMATIONKEYS_H

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
#if Board_SWITCH != PIN_UNASSIGNED
#define HAS_SWITCH
#endif

#if Board_CONTACT != PIN_UNASSIGNED
#define HAS_CONTACT
#endif

#if Board_MOTION != PIN_UNASSIGNED
#define HAS_MOTION
#endif

#if defined(HAS_SWITCH) || defined(HAS_CONTACT) || defined(HAS_MOTION)
#define HAS_KEYS
#endif

/*********************************************************************
 * FUNCTIONS
 */

#ifdef HAS_KEYS
/*
 * Initialize Key module
 */
extern void HomeAutomationKeys_init(void);

/*
 * Task Event Processor for Key module
 */
extern void HomeAutomationKeys_processEvent(void);

/*
 * Reset Key module
 */
extern void HomeAutomationKeys_reset(void);

/*
 * Process switch
 */
#ifdef HAS_SWITCH
extern void HomeAutomationKeys_processKeySwitch(void);
#endif

/*
 * Process contact sensor
 */
#ifdef HAS_CONTACT
extern void HomeAutomationKeys_processKeyContact(void);
#endif

/*
 * Process motion sensor
 */
#ifdef HAS_MOTION
extern void HomeAutomationKeys_processKeyMotion(void);
#endif

#else

/* Keys module not included */
#define HomeAutomationKeys_init()
#define HomeAutomationKeys_processEvent()
#define HomeAutomationKeys_reset()
#define HomeAutomationKeys_processKeySwitch()
#define HomeAutomationKeys_processKeyContact()
#define HomeAutomationKeys_processKeyMotion()

#endif // HAS_KEYS

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* HOMEAUTOMATIONKEYS_H */
