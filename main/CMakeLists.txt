# -*- cmake -*-

set(SOURCE_FILES
    applist.cxx
    bleservice.cxx
    fonts.cxx
    gps.cxx
    hardware.cxx
    main.cxx
    notificationservice.cxx
    task.cxx
    tft.cxx
)

set(REQUIRES
    esp_tinygpsplusplus
    spi_flash
    bt
    esp-nimble-cpp
    esp_driver_uart
    esp_driver_spi
    esp_driver_gpio
    esp_adc
)

idf_component_register(SRCS ${SOURCE_FILES} REQUIRES ${REQUIRES} INCLUDE_DIRS ".")
