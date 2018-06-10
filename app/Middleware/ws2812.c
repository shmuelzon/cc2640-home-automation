/*
 * ws2812.c
 *
 *  Created on: May 14, 2014
 *      Author: a0273433
 *  Taken from: http://processors.wiki.ti.com/index.php/TI-RTOS_WS2812_RGB_LED
 */

#include "ws2812.h"

#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/gates/GateMutex.h>
#include <ti/drivers/SPI.h>
#include <board.h>

/*
 * Local LED frame buffer that is used by the SPI driver. This buffer needs to
 * be be long enough to contain ALL the LEDs that are strung together!
 *
 * Every LED contains 24bits of color data (3 bytes)
 * Every bit of color data is represented as 3 bits of SPI data!
 * Therefore 24bits color data = 3 * 24 = 72 bits of SPI data (9 bytes)
 *
 * To keep things simple, the uDMA transmits 15 bits of SPI data per transfer.
 * This allows the GRB to SPI data conversion to be done exactly with 5 5-bit
 * lookups from the lut4bitSPI lookup table.
 *
 * G[H], G[L], B[H], B[L], R[H], R[L]
 * H: Upper 4 bits of color data
 * L: Lower 4 bits of color data
 *
 * 1 GRB LED = 6 shorts of data.
 */
uint16_t ledFrameBuffer[Board_WS2812_NUM_LEDS * 5];

/*
 * A table to look up the bit pattern that needs to be sent over the SPI line to
 * the LED.
 *
 * The index of the LUT corresponds to 5 bits of data that should be sent to the
 * LED. The returning value represents a 15-bit value which is stored in a short
 * so that the uDMA can easily transfer the data to the SSI transmit register.
 *
 * Bit 15 should be easily ignored by the uDMA
 */
const uint16_t lut4bitSPI[32] = {
    /* 0 */ 0x4924,
    /* 1 */ 0x4926,
    /* 2 */ 0x4934,
    /* 3 */ 0x4936,
    /* 4 */ 0x49A4,
    /* 5 */ 0x49A6,
    /* 6 */ 0x49B4,
    /* 7 */ 0x49B6,
    /* 8 */ 0x4D24,
    /* 9 */ 0x4D26,
    /* A */ 0x4D34,
    /* B */ 0x4D36,
    /* C */ 0x4DA4,
    /* D */ 0x4DA6,
    /* E */ 0x4DB4,
    /* F */ 0x4DB6,
    /*10 */ 0x6924,
    /*11 */ 0x6926,
    /*12 */ 0x6934,
    /*13 */ 0x6936,
    /*14 */ 0x69A4,
    /*15 */ 0x69A6,
    /*16 */ 0x69B4,
    /*17 */ 0x69B6,
    /*18 */ 0x6D24,
    /*19 */ 0x6D26,
    /*1A */ 0x6D34,
    /*1B */ 0x6D36,
    /*1C */ 0x6DA4,
    /*1D */ 0x6DA6,
    /*1E */ 0x6DB4,
    /*1F */ 0x6DB6
};

static GateMutex_Handle frameBuffer;
static SPI_Handle		spiHandle;
/*
 *  ======== WS2812_init ========
 */
void WS2812_init(unsigned int indexSPI)
{
	Error_Block eb;
	SPI_Params	spiParams;
	int indexLED;

	Error_init(&eb);
	frameBuffer = GateMutex_create(NULL, &eb);
	if (!frameBuffer) {
		System_abort("Could not create GateMutex\n");
	}

	SPI_Params_init(&spiParams);
	spiParams.bitRate = 2400000;
	spiParams.dataSize = 15;
	spiParams.frameFormat = SPI_POL0_PHA1;
	spiHandle = SPI_open(indexSPI, &spiParams);

	for (indexLED = 0; indexLED < Board_WS2812_NUM_LEDS; indexLED++) {
		WS2812_setLEDcolor(indexLED, 0,0,0, WS2812_NOREFRESH);
	}
}

/*
 *  ======== WS2812_setLEDcolor ========
 */
bool WS2812_setLEDcolor(uint16_t index, uint8_t g, uint8_t r, uint8_t b, bool refreshLEDS)
{
	uint16_t *ledBuffer = &ledFrameBuffer[index * 5];
	uint16_t tmp[5];
	unsigned int key;

	/*
	 * G        R        B
	 * 76543210 76543210 76543210
	 *
	 *    G
	 * 0: 76543
	 */
	tmp[0] = lut4bitSPI[  g >> 3];

	/*
	 *    G  R
	 * 1: 21076
	 */
	tmp[1] = lut4bitSPI[((g & 0x7) << 2) | (r >> 6)];

	/*    R
	 * 2: 54321
	 */
	tmp[2] = lut4bitSPI[ (r >> 1) & 0x1F];

	/*    RB
	 * 3: 07654
	 */
	tmp[3] = lut4bitSPI[((r & 0x01) << 4) | (b >> 4)];

	/*    B
	 * 4: 3210 + 0 as LSB
	 */
	tmp[4] = lut4bitSPI[ (b & 0xF) << 1] & ~0x0007;

	/*
	 * Use a lock to prevent race conditions if this function is called from
	 * different Tasks.
	 */
	key = GateMutex_enter(frameBuffer);
	ledBuffer[0] = tmp[0];
	ledBuffer[1] = tmp[1];
	ledBuffer[2] = tmp[2];
	ledBuffer[3] = tmp[3];
	ledBuffer[4] = tmp[4];
	GateMutex_leave(frameBuffer, key);

	return (refreshLEDS) ? WS2812_refreshLEDs() : true;
}

/*
 *  ======== WS2812_refreshLEDs ========
 */
bool WS2812_refreshLEDs(void)
{
	SPI_Transaction spiTransaction;

	spiTransaction.count = Board_WS2812_NUM_LEDS * 5;
	spiTransaction.txBuf = ledFrameBuffer;
	spiTransaction.rxBuf = NULL;

	return SPI_transfer(spiHandle, &spiTransaction);
}
