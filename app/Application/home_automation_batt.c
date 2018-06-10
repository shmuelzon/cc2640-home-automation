/******************************************************************************

 @file  home_automation_batt.c

 @brief This file contains the Sensor Tag sample application,
        Battery monitoring sub-task.

 Group: WCS, BTS
 Target Device: CC2650, CC2640

 ******************************************************************************
 
 Copyright (c) 2016-2018, Texas Instruments Incorporated
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

 ******************************************************************************
 Release Name: ble_sdk_2_02_02_25
 Release Date: 2018-04-02 18:03:35
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include "board.h"
#include "gatt.h"
#include "gattservapp.h"
#include "util.h"

#include "battservice.h"
#include "home_automation_batt.h"
#include "home_automation.h"

#ifndef EXCLUDE_BATT

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// How often to check battery (milliseconds)
#define BATT_PERIOD         86400000 /* 1 day */

// Battery level is critical when it is less than this %
#define BATT_CRITICAL_LEVEL 60

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
static Clock_Struct periodicClock;
static bool sensorReadScheduled;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void sensorConfigChangeCB(uint8_t paramID);
static void HomeAutomationBatt_clockHandler(UArg arg);

/*********************************************************************
 * PUBLIC FUNCTIONS
 */


/*********************************************************************
 * @fn      HomeAutomationBatt_init
 *
 * @brief   Initialize scheduler for battery monitoring
 *
 * @param   none
 *
 * @return  none
 */
void HomeAutomationBatt_init(void)
{
  // Add battery service.
  Batt_AddService();
  Batt_Setup(Board_BATT_MIN, Board_BATT_MAX, NULL, NULL);

  // Register for Battery service callback.
  Batt_Register(&sensorConfigChangeCB);

  // Initialize the module state variables
  HomeAutomationBatt_reset();

  // Create periodic clock for internal battery check event
  Util_constructClock(&periodicClock, HomeAutomationBatt_clockHandler,
    100, BATT_PERIOD, false, 0);
}

/*********************************************************************
 * @fn      HomeAutomationBatt_processCharChangeEvt
 *
 * @brief   Home automation battery monitor event handling
 *
 * @param   event - event identifier
 *
 */
void HomeAutomationBatt_processCharChangeEvt(uint8_t event)
{
  if (event == BATT_LEVEL_NOTI_ENABLED)
    Util_startClock(&periodicClock);
  else if (event == BATT_LEVEL_NOTI_DISABLED)
    Util_stopClock(&periodicClock);
}

/*********************************************************************
 * @fn      HomeAutomationBatt_reset
 *
 * @brief   Reset characteristics and disable sensor
 *
 * @param   none
 *
 * @return  none
 */
void HomeAutomationBatt_reset(void)
{
    // Setup Battery Characteristic Values.
    uint8_t critical = BATT_CRITICAL_LEVEL;
    Batt_SetParameter(BATT_PARAM_CRITICAL_LEVEL, sizeof(uint8_t), &critical);

    // Clear any scheduled read
    sensorReadScheduled = false;

    // Make sure clock stops
    Util_stopClock(&periodicClock);
}

/*********************************************************************
* Private functions
*/

/*********************************************************************
 * @fn      HomeAutomationBatt_processSensorEvent
 *
 * @brief   Home automation battery monitor event processor.
 *
 */
void HomeAutomationBatt_processSensorEvent(void)
{
    if (sensorReadScheduled)
    {
        sensorReadScheduled = false;

        // Perform battery level check.
        Batt_MeasLevel();
    }
}

/*********************************************************************
 * @fn      HomeAutomationBatt_clockHandler
 *
 * @brief   Handler function for clock time-outs.
 *
 * @param   arg - event type
 *
 * @return  none
 */
static void HomeAutomationBatt_clockHandler(UArg arg)
{
  sensorReadScheduled = true;

  // Wake up the application.
  Semaphore_post(sem);
}


/*********************************************************************
 * @fn      sensorConfigChangeCB
 *
 * @brief   Callback from Battery Service indicating a configuration change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void sensorConfigChangeCB(uint8_t paramID)
{
  // Wake up the application thread
  HomeAutomation_charValueChangeCB(SERVICE_ID_BATT, paramID);
}
#endif // EXCLUDE_BATT

/*********************************************************************
*********************************************************************/
