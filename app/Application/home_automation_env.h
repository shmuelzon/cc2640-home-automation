#ifndef HOMEAUTOMATIONENV_H
#define HOMEAUTOMATIONENV_H

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
#if Board_OPT3001_ADDR != 0
#define HAS_OPT3001
#endif

#if Board_BME280_ADDR != 0
#define HAS_BME280
#endif

#if defined(HAS_OPT3001) || defined(HAS_BME280) || Board_THERMISTOR != PIN_UNASSIGNED
#define HAS_ENV
#endif

/*********************************************************************
 * FUNCTIONS
 */

#ifdef HAS_ENV
/*
 * Initialize Env module
 */
extern void HomeAutomationEnv_init(void);

/*
 * Task Event Processor for Env module
 */
extern void HomeAutomationEnv_processEvent(void);

/*
 * Reset Env module
 */
extern void HomeAutomationEnv_reset(void);

#else

/* Env module not included */
#define HomeAutomationEnv_init()
#define HomeAutomationEnv_processEvent()
#define HomeAutomationEnv_reset()

#endif // HAS_ENV

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* HOMEAUTOMATIONENV_H */
