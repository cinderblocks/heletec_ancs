# -*- cmake -*-

set(SOURCE_FILES
    heltec.cxx
    ancsservice.cxx
    notificationservice.cxx
    applist.cxx
    security.cxx
    task.cxx
    main.cxx
)

set(REQUIRES
    arduino
    heltec
    spi_flash
)

idf_component_register(SRCS ${SOURCE_FILES} REQUIRES ${REQUIRES} INCLUDE_DIRS ".")
