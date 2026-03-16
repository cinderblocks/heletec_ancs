#pragma once
// Minimal FreeRTOS host stub — just enough to compile ESP-IDF code on a POSIX host.
#include <stdint.h>   // uint*_t, uintptr_t
#include <stddef.h>   // size_t

typedef unsigned int TickType_t;
typedef int          BaseType_t;
typedef unsigned int UBaseType_t;

#define pdTRUE  ((BaseType_t)1)
#define pdFALSE ((BaseType_t)0)
#define pdPASS  pdTRUE
#define pdFAIL  pdFALSE
#define portMAX_DELAY   ((TickType_t)0xFFFFFFFFU)
#define configTICK_RATE_HZ 100

static inline TickType_t xTaskGetTickCount(void) { return 0; }