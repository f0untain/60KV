#ifndef __BUTTON_H_
#define __BUTTON_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/**
 * @brief Number of states per cycle for rotary encoders.
 * @details Set to 2 for ALPS rotary encoders and 4 for others.
 */
#define ROTARY_SPC          2

/**
 * @brief Constant indicating clockwise direction for rotary encoders.
 */
#define ROTARY_CW_DIR       0x10

/**
 * @brief Constant indicating counter-clockwise direction for rotary encoders.
 */
#define ROTARY_CCW_DIR      0x20

/**
 * @brief Key definitions representing specific button actions.
 */
/*#define KEY_NULL			0
#define KEY_START			0x01
#define KEY_ACCEPT			0x02
#define KEY_BACK			0x04
#define KEY_STANDBY		    0x08*/

//New keypad board pin & New micro board
#define KEY_NULL			0
#define KEY_ACCEPT			0x01
#define KEY_START			0x02
#define KEY_BACK			0x04
#define KEY_STANDBY		    0x08

/**
 * @brief GPIO pin masks for buttons.
 */
#define KEY_GPIOA_MASK      (KeyBack_Pin | KeyStandby_Pin | KeyRotaryMiddle_Pin) ///< Mask for GPIOA pins.
#define KEY_GPIOC_MASK      (KeyStart_Pin) ///< Mask for GPIOC pins.

/**
 * @brief Macro to read all button states and pack them into a byte.
 * @details Shifts key states to bit 0-4 of a byte.
 */
#define KEY_READ            (((uint8_t)((GPIOA->IDR & KEY_GPIOA_MASK)>>10)&0x07) | ((uint8_t)((GPIOC->IDR & KEY_GPIOC_MASK)>>11)&0x08))

/**
 * @brief Mask for valid key states.
 */
#define KEY_MASK            0x0F

/**
 * @brief Mask for buttons that trigger an action when released.
 */

#define KEY_RELEASE_MASK    0x0F

/* Bits is set to one if a debounced release is detected. */
extern volatile uint8_t buttonRelease;

void Debounce(void);
void ReadRotaryMiddleKey();
void ReadStandbyKey();
void ReadBackKey();
void ReadStartKey();
void VcDecOrSet(uint8_t high, uint8_t low, uint8_t mask);
	
	
#ifdef __cplusplus
}
#endif

#endif  /* __BUTTON_H */






