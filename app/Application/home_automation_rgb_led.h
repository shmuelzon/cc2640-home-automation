#ifndef HOMEAUTOMATIONRGBLED_H
#define HOMEAUTOMATIONRGBLED_H

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

#if Board_RGB_NUM_LEDS > 0
/*
 * Initialize RGB LED module
 */
extern void HomeAutomationRGBLED_init(void);

/*
 * Task Event Processor for characteristic changes
 */
void HomeAutomationRGBLED_processCharChangeEvt(uint8_t paramID);

/*
 * Task Event Processor for RGB LED module
 */
extern void HomeAutomationRGBLED_processEvent(void);

/*
 * Reset RGB LED module
 */
extern void HomeAutomationRGBLED_reset(void);

/*
 * Set specific LED color
 */
extern void HomeAutomationRGBLED_SetLedColor(uint16_t index, uint8_t r, uint8_t g, uint8_t b);

/*
 * Update LED colors
 */
extern void HomeAutomationRGBLED_Update(void);

#else

/* RGB LED module not included */
#define HomeAutomationRGBLED_init()
#define HomeAutomationRGBLED_processCharChangeEvt(paramID)
#define HomeAutomationRGBLED_processEvent()
#define HomeAutomationRGBLED_reset()
#define HomeAutomationRGBLED_SetLedColor(index, r, g, b)
#define HomeAutomationRGBLED_Update()

#endif // Board_RGB_NUM_LEDS > 0

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* HOMEAUTOMATIONRGBLED_H */
