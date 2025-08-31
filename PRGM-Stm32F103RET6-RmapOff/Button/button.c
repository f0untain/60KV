#include "button.h"

volatile uint8_t buttonRelease;

/**
 * @brief Functions to read the current state of specific buttons.
 */
void ReadRotaryMiddleKey(void) { HAL_GPIO_ReadPin(KeyRotaryMiddle_GPIO_Port, KeyRotaryMiddle_Pin); } // Read middle key state.
void ReadStandbyKey(void) { HAL_GPIO_ReadPin(KeyStandby_GPIO_Port, KeyStandby_Pin); }                // Read standby key state.
void ReadBackKey(void) { HAL_GPIO_ReadPin(KeyBack_GPIO_Port, KeyBack_Pin); }                         // Read back key state.
void ReadStartKey(void) { HAL_GPIO_ReadPin(KeyStart_GPIO_Port, KeyStart_Pin); }                      // Read start key state.

/**
 * @brief Decrease 2-bit vertical counter where mask = 1, or set counters to 0b11 where mask = 0.
 * @param high High byte of the counter.
 * @param low Low byte of the counter.
 * @param mask Mask indicating which counters to decrease or set.
 */
void VcDecOrSet(uint8_t high, uint8_t low, uint8_t mask) {
    low = ~(low & mask);
	high = low ^ (high & mask);
}

/**
* @brief Debounces up to 8 buttons connected to a port.
*
* @details This function checks the state of the buttons and updates the `buttonsButtonRelease` variable
* if a debounced button press or ButtonRelease is detected. It should be called approximately 100 times per second.
*/
void Debounce(void)
{
	/**
	* @brief Eight vertical 2-bit counters to count the number of equal states.
	*/
	static uint8_t vcountLow  = 0xFF, vcountHigh = 0xFF;

	/**
	* @brief Tracks the current (debounced) state of the buttons.
	*/
	static uint8_t buttonState = 0;

	/**
	* @brief XOR with `buttonState` to detect state changes.
	*/
	uint8_t stateChanged = (KEY_READ) ^ buttonState;

	/* Decrease counters where stateChanged = 1, set the others to 0b11. */
	VcDecOrSet(vcountHigh, vcountLow, stateChanged);

	/* Update stateChanged to have a 1 only if the counter overflowed. */
	stateChanged &= vcountLow & vcountHigh;

	/* Change buttonState for the buttons whose counters rolled over. */
	buttonState ^= stateChanged;

	/* Update buttonsButtonRelease with buttons whose counters rolled over
	* and are still in state 0 (ButtonReleased).
	*/
	buttonRelease |= (buttonState & (KEY_RELEASE_MASK)) & stateChanged; // ButtonRelease action.
}
