#pragma once
#include "FreeRTOS.h"
// portMUX_TYPE — used by portENTER_CRITICAL / portEXIT_CRITICAL
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
static inline void portMUX_INITIALIZE(portMUX_TYPE* m) { (void)m; }
#define portENTER_CRITICAL(m) (void)(m)
#define portEXIT_CRITICAL(m)  (void)(m)
#define IRAM_ATTR
