#pragma once
#include "FreeRTOS.h"
typedef void* TaskHandle_t;
static inline TaskHandle_t xTaskGetCurrentTaskHandle(void) { return (void*)1; }
static inline void vTaskDelay(TickType_t t) { (void)t; }
static inline void vTaskDelete(TaskHandle_t t) { (void)t; }
static inline uint32_t ulTaskNotifyTake(int c, TickType_t t) { (void)c;(void)t; return 0; }
static inline BaseType_t xTaskNotify(TaskHandle_t t, uint32_t v, int a) { (void)t;(void)v;(void)a; return pdTRUE; }
static inline BaseType_t xTaskNotifyWait(uint32_t a, uint32_t b, uint32_t* v, TickType_t t)
    { (void)a;(void)b;if(v)*v=0;(void)t; return pdTRUE; }
static inline BaseType_t xTaskCreatePinnedToCore(void(*f)(void*),const char* n,uint32_t s,void* p,UBaseType_t pr,TaskHandle_t* h,int c)
    { (void)f;(void)n;(void)s;(void)p;(void)pr;(void)c;if(h)*h=(void*)1;return pdTRUE; }
typedef void* TimerHandle_t;
static inline TimerHandle_t xTimerCreate(const char* n,TickType_t p,int r,void* id,void(*cb)(TimerHandle_t))
    { (void)n;(void)p;(void)r;(void)id;(void)cb;return (void*)1; }
static inline BaseType_t xTimerStart(TimerHandle_t t, TickType_t w) { (void)t;(void)w; return pdTRUE; }
static inline BaseType_t xTimerStop(TimerHandle_t t, TickType_t w)  { (void)t;(void)w; return pdTRUE; }
static inline BaseType_t xTimerReset(TimerHandle_t t, TickType_t w) { (void)t;(void)w; return pdTRUE; }
static inline void* pvTimerGetTimerID(TimerHandle_t t) { (void)t; return nullptr; }
#define eSetBits 2
