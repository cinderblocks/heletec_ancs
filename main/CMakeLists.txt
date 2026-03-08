# -*- cmake -*-

set(SOURCE_FILES
    applist.cxx
    bleservice.cxx
    fonts.cxx
    gps.cxx
    hardware.cxx
    main.cxx
    notificationservice.cxx
    security.cxx
    task.cxx
    tft.cxx
)

set(REQUIRES
    arduino-esp32
    Heltec_ESP32
    spi_flash
    bt
    esp-nimble-cpp
    esp_driver_uart
    esp_driver_spi
)

idf_component_register(SRCS ${SOURCE_FILES} REQUIRES ${REQUIRES} INCLUDE_DIRS ".")
