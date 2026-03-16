#pragma once
#include "FreeRTOS.h"
typedef void* QueueHandle_t;
static inline QueueHandle_t xQueueCreate(UBaseType_t l, UBaseType_t s) { (void)l;(void)s; return (void*)1; }
static inline BaseType_t xQueueSend(QueueHandle_t q, const void* i, TickType_t t) { (void)q;(void)i;(void)t; return pdTRUE; }
static inline BaseType_t xQueueReceive(QueueHandle_t q, void* i, TickType_t t) { (void)q;(void)i;(void)t; return pdFALSE; }
static inline void xQueueReset(QueueHandle_t q) { (void)q; }
static inline void vQueueDelete(QueueHandle_t q) { (void)q; }
static inline BaseType_t xQueueSendFromISR(QueueHandle_t q, const void* i, BaseType_t* pw) { (void)q;(void)i;(void)pw; return pdTRUE; }
