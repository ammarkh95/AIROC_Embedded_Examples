/** @file
*
* LE LED Controller UUID
*
* This file provides UUID definitions for LED Controller
*
*/
#ifndef _LED_UUID_H_
#define _LED_UUID_H_
/******************************************************************************
 *                          Constants
 ******************************************************************************/

/* 16-bit UUID of the LED Controller Service */
#define UUID_LED_SERVICE  0x23, 0x20, 0x56, 0x7c, 0x05, 0xcf, 0x6e, 0xb4, 0xc3, 0x41, 0x77, 0x28, 0x51, 0x82, 0x7e, 0x1b

/* 16-bit UUID value of the LED Controller ON / OFF Characteristic Configuration */
#define UUID_LED_CHARACTERISTIC_ON_OFF_CONFIG 0x1a, 0x89, 0x07, 0x4a, 0x2f, 0x3b, 0x7e, 0xa6, 0x81, 0x44, 0x3f, 0xf9, 0xa8, 0xf2, 0x9b, 0x5e

/* 16-bit UUID value of the LED Controller Blink period Characteristic, Configuration */
#define UUID_LED_CHARACTERISTIC_BLINK_PERIOD_CONFIG 0x1b, 0x88, 0x06, 0x3a, 0x2e, 0x2b, 0x4e, 0xa5, 0x71, 0x33, 0x3a, 0xf7, 0xb8, 0xc2, 0x1b, 0x6e


#endif // _LED_UUID_H_
