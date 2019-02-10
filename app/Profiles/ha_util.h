/******************************************************************************

 @file  ha_util.h

 @brief Utilities for Home Automation services

 Group: WCS, BTS
 Target Device: CC2650, CC2640

 ******************************************************************************

 Copyright (c) 2012-2018, Texas Instruments Incorporated
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

#ifndef HA_UTIL_H
#define HA_UTIL_H

#include "gatt.h"

/*********************************************************************
 * MACROS
 */
#ifndef GATT_HA_UUID_16_BIT

// HA Base 128-bit UUID: 1E35XXXX-1217-5887-A357-C9ACBD789B14
#define HA_BASE_UUID_128(uuid) 0x14, 0x9B, 0x78, 0xBD, 0xAC, 0xC9, 0x57, 0xA3, \
  0x87, 0x58, 0x17, 0x12, LO_UINT16(uuid), HI_UINT16(uuid), 0x35, 0x1E

#define HA_UUID_SIZE ATT_UUID_SIZE
#define HA_UUID(uuid) HA_BASE_UUID_128(uuid)

#else

// Using 16-bit UUID
#define HA_UUID_SIZE ATT_BT_UUID_SIZE
#define HA_UUID(uuid) LO_UINT16(uuid), HI_UINT16(uuid)

#endif

/*-------------------------------------------------------------------
 * FUNCTIONS
 */

extern bStatus_t utilExtractUuid16(gattAttribute_t *pAttr, uint16_t *pValue);

#endif /* HA_UTIL_H */
