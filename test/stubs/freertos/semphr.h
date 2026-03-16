#pragma once
#include "FreeRTOS.h"
typedef void* SemaphoreHandle_t;
// Return a stable non-null sentinel address without allocating anything.
// reinterpret_cast of an integer constant is well-defined as a non-null
// pointer and avoids static variable duplication across TUs.
static inline SemaphoreHandle_t xSemaphoreCreateMutex(void) { return reinterpret_cast<void*>(static_cast<uintptr_t>(1)); }
static inline BaseType_t xSemaphoreTake(SemaphoreHandle_t s, TickType_t t) { (void)s;(void)t; return pdTRUE; }
static inline BaseType_t xSemaphoreGive(SemaphoreHandle_t s) { (void)s; return pdTRUE; }
static inline void vSemaphoreDelete(SemaphoreHandle_t s) { (void)s; }