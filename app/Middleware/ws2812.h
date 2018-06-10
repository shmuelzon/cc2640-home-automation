/*
 * ws2812.h
 *
 *  Created on: May 14, 2014
 *      Author: a0273433
 *  Taken from: http://processors.wiki.ti.com/index.php/TI-RTOS_WS2812_RGB_LED
 */

#ifndef WS2812_H_
#define WS2812_H_

#ifndef LED_COUNT
#define LED_COUNT   		24
#endif

#define WS2812_REFRESH		true
#define WS2812_NOREFRESH	false

#include <stdint.h>
#include <stdbool.h>

void WS2812_init(unsigned int indexSPI);

bool WS2812_setLEDcolor(uint16_t index, uint8_t g, uint8_t r, uint8_t b, bool refreshLEDS);

bool WS2812_refreshLEDs(void);

#endif /* WS2812_H_ */
