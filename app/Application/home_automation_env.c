/*********************************************************************
 * INCLUDES
 */
#include "gatt.h"
#include "gattservapp.h"
#include "home_automation_env.h"
#include "board.h"
#include "peripheral.h"
#include "util.h"
#include "SensorOpt3001.h"
#include "SensorBme280.h"
#include "environmentalsensingservice.h"

#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>

#ifdef HAS_ENV
/*********************************************************************
 * CONSTANTS
 */
// How often should the sensors be polled (in milliseconds)
#define ENV_POLL_TIME            60000 // One minute

// Events
#define ENV_POLL_EVT             (1 << 0)
#define ENV_DATA_READY_EVT       (1 << 1)

/*********************************************************************
 * LOCAL VARIABLES
 */
static Clock_Struct periodPoll;
static Clock_Struct periodDataReady;
static uint8_t events;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void HomeAutomationEnv_clockHandler(UArg arg);
static void HomeAutomationEnv_pollSensors(void);
static void HomeAutomationEnv_dataReady(void);

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      HomeAutomationEnv_init
 *
 * @brief   Initialization function for the HomeAutomation env sensors
 *
 * @param   none
 *
 * @return  none
 */
void HomeAutomationEnv_init(void)
{
  uint32_t services = 0;
  uint16_t convTime = 0;

#ifdef HAS_OPT3001
  services |= ESS_SERVICE_LUMINANCE;
  SensorOpt3001_init();
  convTime = MAX(convTime, SensorOpt3001_measurementTime());
#endif
#ifdef HAS_BME280
  services |= ESS_SERVICE_PRESSURE | ESS_SERVICE_TEMPERATURE | ESS_SERVICE_HUMIDITY;
  SensorBme280_init();
  convTime = MAX(convTime, SensorBme280_measurementTime());
#endif
  // Add relevant services
  EnvironmentalSensing_AddService(services);

  // Initialize the module state variables
  HomeAutomationEnv_reset();

  // Create one-shot clock for sensor polling
  Util_constructClock(&periodPoll, HomeAutomationEnv_clockHandler,
    ENV_POLL_TIME - convTime, 0, false, ENV_POLL_EVT);
  // Create one-shot clock for key press latching
  Util_constructClock(&periodDataReady, HomeAutomationEnv_clockHandler,
    convTime, 0, false, ENV_DATA_READY_EVT);

  // Start polling the sensors
  HomeAutomationEnv_pollSensors();
}

/*********************************************************************
 * @fn      HomeAutomationEnv_processEvent
 *
 * @brief   HomeAutomation Env event processor.
 *
 * @param   none
 *
 * @return  none
 */
void HomeAutomationEnv_processEvent(void)
{
  if (events & ENV_POLL_EVT)
  {
    events &= ~ENV_POLL_EVT;
    HomeAutomationEnv_pollSensors();
  }

  if (events & ENV_DATA_READY_EVT)
  {
    events &= ~ENV_DATA_READY_EVT;
    HomeAutomationEnv_dataReady();
  }
}

/*********************************************************************
 * @fn      HomeAutomationEnv_reset
 *
 * @brief   Reset key state
 *
 * @param   none
 *
 * @return  none
 */
void HomeAutomationEnv_reset(void)
{
#ifdef HAS_OPT3001
  SensorOpt3001_enable(false);
#endif
#ifdef HAS_BME280
  SensorBme280_enable(false);
#endif
  events = 0;
}

/*********************************************************************
 * @fn      HomeAutomationEnv_clockHandler
 *
 * @brief   Handler function for clock time-outs.
 *
 * @param   arg - event type
 *
 * @return  none
 */
static void HomeAutomationEnv_clockHandler(UArg arg)
{
  // Store the event.
  events |= arg;

  // Wake up the application.
  Semaphore_post(sem);
}

/*********************************************************************
 * @fn      HomeAutomationEnv_pollSensors
 *
 * @brief   Start polling the sensors.
 *
 * @return  none
 */
static void HomeAutomationEnv_pollSensors(void)
{
#ifdef HAS_OPT3001
  SensorOpt3001_enable(true);
#endif
#ifdef HAS_BME280
  SensorBme280_enable(true);
#endif
  Util_startClock(&periodDataReady);
}

/*********************************************************************
 * @fn      HomeAutomationEnv_dataReady
 *
 * @brief   Retrieve results from the sensors.
 *
 * @return  none
 */
static void HomeAutomationEnv_dataReady(void)
{
#ifdef HAS_OPT3001
  {
    float res;
    uint32_t tmp;

    if (SensorOpt3001_read(&res))
    {
      tmp = res * 100;
      EnvironmentalSensing_SetParameter(LUMINANCE_PARAM_VALUE, sizeof(tmp), &tmp);
    }

    SensorOpt3001_enable(false);
  }
#endif
#ifdef HAS_BME280
  {
    int16_t tmp;
    uint16_t tmp2;
    int32_t temp;
    uint32_t press, hum;

    if (SensorBme280_read(&temp, &press, &hum))
    {
      tmp = (int16_t)temp; // The temperature characteristic expects signed-16bit integer
      EnvironmentalSensing_SetParameter(TEMPERATURE_PARAM_VALUE, sizeof(tmp), &tmp);
      press *= 10; // The pressure characteristic expects a resolution of 0.1Pa
      EnvironmentalSensing_SetParameter(PRESSURE_PARAM_VALUE, sizeof(press), &press);
      tmp2 = hum / 1024.0 * 100; // Convert Q22.10 to 0.01 resolution
      EnvironmentalSensing_SetParameter(HUMIDITY_PARAM_VALUE, sizeof(tmp2), &tmp2);
    }

    SensorBme280_enable(false);
  }
#endif
  Util_startClock(&periodPoll);
}

#endif // HAS_ENV

/*********************************************************************
*********************************************************************/
