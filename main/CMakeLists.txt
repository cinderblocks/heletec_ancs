# -*- cmake -*-

set(SOURCE_FILES
    applist.cxx
    applistservice.cxx
    bleservice.cxx
    buzzer.cxx
    diag.cxx
    fonts.cxx
    gps.cxx
    hardware.cxx
    lora.cxx
    main.cxx
    meshnode.cxx
    meshtastic_proto.cxx
    notificationservice.cxx
    sx1262.cxx
    task.cxx
    tft.cxx
)

set(REQUIRES
    esp_tinygpsplusplus
    spi_flash
    nvs_flash
    bt
    esp-nimble-cpp
    esp_driver_uart
    esp_driver_spi
    esp_driver_gpio
    esp_driver_ledc
    esp_adc
    mbedtls
)

idf_component_register(SRCS ${SOURCE_FILES} REQUIRES ${REQUIRES} INCLUDE_DIRS ".")
