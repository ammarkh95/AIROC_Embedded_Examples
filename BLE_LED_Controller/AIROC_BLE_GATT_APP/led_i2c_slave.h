/** @file
*
* ESP 32 I2C LED Control Interface  
*
* This file provides I2C definitions for ESP32 slave device to control its built-in LED on GPIO5 (DevKit C)
*
*/
#ifndef _ESP32_I2C_H_
#define _ESP32_I2C_H_
/******************************************************************************
 *                          Constants
 ******************************************************************************/

/* I2C identification address of the ESP32 */
#define ESP32_I2C_ADDRESS  0x25

/* I2C command to turn on ESP32 built-in LED on GPIO5 */
#define LED_ON_COMMAND  0x10

/* I2C command to turn off ESP32 built-in LED on GPIO5 */
#define LED_OFF_COMMAND  0x20

/* I2C command to set the blinking period of the LED */
#define LED_SET_BLINK_PERIOD_COMMAND  0x30


#endif // _ESP32_I2C_H_
