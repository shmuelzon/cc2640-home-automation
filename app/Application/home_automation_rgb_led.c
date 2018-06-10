/*********************************************************************
 * INCLUDES
 */
#include "osal_snv.h"
#include "gatt.h"
#include "gattservapp.h"
#include "home_automation_rgb_led.h"
#include "board.h"
#include "peripheral.h"
#include "util.h"
#include "rgbledservice.h"
#include "ws2812.h"

#if Board_RGB_NUM_LEDS > 0

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
// Events
#define RGB_LED_COLOR_CHANGED_EVT            (1 << 0)

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
static struct {
  uint8_t r;
  uint8_t g;
  uint8_t b;
} __attribute__((packed)) colors[Board_RGB_NUM_LEDS] = {};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void HomeAutomationRGBLED_StateChangeCB(void);

/*********************************************************************
 * PROFILE CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      HomeAutomationRGBLED_init
 *
 * @brief   Initialization function for the RGB LEDs
 *
 * @param   none
 *
 * @return  none
 */
void HomeAutomationRGBLED_init(void)
{
  uint16_t i;

  // Add RGB LED service
  RGBLED_AddService();
  RGBLED_Setup(HomeAutomationRGBLED_StateChangeCB);

#if Board_WS2812_NUM_LEDS > 0
  WS2812_init(Board_WS2812_SPI);
#endif

  // Turn off all LEDs
  for (i = 0; i < Board_RGB_NUM_LEDS; i++)
    HomeAutomationRGBLED_SetLedColor(i, 0, 0, 0);
  HomeAutomationRGBLED_Update();

  // Initialize the module state variables
  HomeAutomationRGBLED_reset();
}

/*********************************************************************
 * @fn      HomeAutomationRGBLED_processCharChangeEvt
 *
 * @brief   HomeAutomation RGB LED event handling
 *
 * @param   event - event identifier
 *
 */
void HomeAutomationRGBLED_processCharChangeEvt(uint8_t event)
{
  if (event == RGB_LED_COLOR_CHANGED_EVT)
  {
    uint8_t i, color[3 * Board_RGB_NUM_LEDS];

    RGBLED_GetParameter(RGB_LED_PARAM_COLOR, &color);
    /* Set LED colors */
    for (i = 0; i < Board_RGB_NUM_LEDS; i++)
      HomeAutomationRGBLED_SetLedColor(i, color[i * 3], color[(i * 3) + 1], color[(i * 3) + 2]);
    /* Update */
    HomeAutomationRGBLED_Update();
  }
}

/*********************************************************************
 * @fn      HomeAutomationRGBLED_processEvent
 *
 * @brief   RGB LED event processor.
 *
 * @param   none
 *
 * @return  none
 */
void HomeAutomationRGBLED_processEvent(void)
{
}

/*********************************************************************
 * @fn      HomeAutomationRGBLED_reset
 *
 * @brief   Reset RGB LED
 *
 * @param   none
 *
 * @return  none
 */
void HomeAutomationRGBLED_reset(void)
{
  uint16_t i;

  // Set all LEDs to internal state
  for (i = 0; i < Board_RGB_NUM_LEDS; i++)
    HomeAutomationRGBLED_SetLedColor(i, colors[i].r, colors[i].g, colors[i].b);

  HomeAutomationRGBLED_Update();
}

/*********************************************************************
 * @fn      HomeAutomationRGBLED_StateChangeCB
 *
 * @brief   Callback function to be called when RGB LEDs are changed.
 *
 * @return  None.
 */
static void HomeAutomationRGBLED_StateChangeCB(void)
{
  // Wake up the application thread
  HomeAutomation_charValueChangeCB(SERVICE_ID_RGBLED, RGB_LED_COLOR_CHANGED_EVT);
}

/*********************************************************************
 * @fn      HomeAutomationRGBLED_SetLedColor
 *
 * @brief   Set a specific LED's color.
 *
 * @param   on - the state to set the relay.
 * @param   r - the red portion of the color.
 * @param   g - the green portion of the color.
 * @param   b - the blue portion of the color.
 *
 * @return  None.
 */
void HomeAutomationRGBLED_SetLedColor(uint16_t index, uint8_t r, uint8_t g, uint8_t b)
{
  colors[index].r = r;
  colors[index].g = g;
  colors[index].b = b;
}

/*********************************************************************
 * @fn      HomeAutomationRGBLED_Update
 *
 * @brief   Send color configuration to LEDs.
 *
 * @return  None.
 */
void HomeAutomationRGBLED_Update(void)
{
  uint16_t i = 0, handled = 0;

#if Board_WS2812_NUM_LEDS > 0
  for (; i - handled < Board_WS2812_NUM_LEDS; i++)
    WS2812_setLEDcolor(i - handled, colors[i].g, colors[i].r, colors[i].b, WS2812_NOREFRESH);
  WS2812_refreshLEDs();
  handled += Board_WS2812_NUM_LEDS;
#endif

  RGBLED_SetParameter(RGB_LED_PARAM_COLOR, sizeof(colors), colors);
}

#endif // Board_RGB_NUM_LEDS > 0

/*********************************************************************
*********************************************************************/
