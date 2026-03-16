#pragma once
#include "esp_log.h"
#define ESP_ERR_NVS_NOT_FOUND       0x1102
#define ESP_ERR_NVS_NO_FREE_PAGES   0x1103
#define ESP_ERR_NVS_NEW_VERSION_FOUND 0x1104

typedef uint32_t nvs_handle_t;
typedef int      nvs_open_mode_t;
#define NVS_READONLY  0
#define NVS_READWRITE 1

static inline esp_err_t nvs_flash_init(void) { return ESP_OK; }
static inline esp_err_t nvs_flash_erase(void) { return ESP_OK; }
static inline esp_err_t nvs_open(const char* ns, nvs_open_mode_t m, nvs_handle_t* h)
    { (void)ns;(void)m;if(h)*h=0; return ESP_ERR_NVS_NOT_FOUND; }
static inline void      nvs_close(nvs_handle_t h) { (void)h; }
static inline esp_err_t nvs_get_u8(nvs_handle_t h, const char* k, uint8_t* v)
    { (void)h;(void)k;if(v)*v=0; return ESP_ERR_NVS_NOT_FOUND; }
static inline esp_err_t nvs_set_u8(nvs_handle_t h, const char* k, uint8_t v)
    { (void)h;(void)k;(void)v; return ESP_OK; }
static inline esp_err_t nvs_get_blob(nvs_handle_t h, const char* k, void* d, size_t* l)
    { (void)h;(void)k;(void)d;(void)l; return ESP_ERR_NVS_NOT_FOUND; }
static inline esp_err_t nvs_set_blob(nvs_handle_t h, const char* k, const void* d, size_t l)
    { (void)h;(void)k;(void)d;(void)l; return ESP_OK; }
static inline esp_err_t nvs_erase_all(nvs_handle_t h) { (void)h; return ESP_OK; }
static inline esp_err_t nvs_commit(nvs_handle_t h)    { (void)h; return ESP_OK; }
