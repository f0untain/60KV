#include "input.h"
#include "menu.h"

DebounceState debounceState = INPUT_IDLE;
uint32_t debounceStart = 0;

void CheckInputDebounced(void)
{
    switch (debounceState)
    {
        case INPUT_IDLE:
            if (HAL_GPIO_ReadPin(ShortCircuitDetect_GPIO_Port, ShortCircuitDetect_Pin) == GPIO_PIN_SET)
            {
                debounceState = INPUT_WAIT;
                debounceStart = HAL_GetTick();
            }
            break;

        case INPUT_WAIT:
            if (HAL_GPIO_ReadPin(ShortCircuitDetect_GPIO_Port, ShortCircuitDetect_Pin) == GPIO_PIN_SET)
            {
                if ((HAL_GetTick() - debounceStart) >= 10)  // 10ms debounce
                {
                    debounceState = INPUT_CONFIRMED;
                }
            }
            else
            {
                debounceState = INPUT_IDLE;  //It was noise
            }
            break;

        case INPUT_CONFIRMED:
        	SetErrorState(SHORTCIRCUIT);
            //error_state = SHORTCIRCUIT;
            debounceState = INPUT_IDLE;
            break;
    }
}
