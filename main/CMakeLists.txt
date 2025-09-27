# -*- cmake -*-

set(SOURCE_FILES
    ancsservice.cxx
    applist.cxx
    fonts.cxx
    gps.cxx
    heltec.cxx
    main.cxx
    notificationservice.cxx
    security.cxx
    task.cxx
    tft.cxx
)

set(REQUIRES
    arduino
    heltec
    spi_flash
)

idf_component_register(SRCS ${SOURCE_FILES} REQUIRES ${REQUIRES} INCLUDE_DIRS ".")
