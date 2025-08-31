#ifndef __INPUT_H_
#define __INPUT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef enum {
    INPUT_IDLE,
    INPUT_WAIT,
    INPUT_CONFIRMED
} DebounceState;

void CheckInputDebounced(void);

#ifdef __cplusplus
}
#endif

#endif  /* __INPUT_H */






