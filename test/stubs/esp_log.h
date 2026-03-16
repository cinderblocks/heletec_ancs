#pragma once
#include <stdio.h>
#define ESP_OK               0
#define ESP_FAIL            -1
#define ESP_ERR_INVALID_ARG 0x102
#define ESP_ERR_INVALID_STATE 0x103

#define ESP_LOGI(tag, fmt, ...) (void)(tag)
#define ESP_LOGW(tag, fmt, ...) (void)(tag)
#define ESP_LOGE(tag, fmt, ...) (void)(tag)
#define ESP_LOGD(tag, fmt, ...) (void)(tag)
#define ESP_EARLY_LOGE(tag, fmt, ...) (void)(tag)

typedef int esp_err_t;
static inline const char* esp_err_to_name(esp_err_t e) { (void)e; return "ERR"; }
