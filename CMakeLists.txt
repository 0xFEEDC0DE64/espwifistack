set(headers
    src/espwifistack.h
    src/espwifistackconfig.h
    src/espwifistackenums.h
    src/espwifiutils.h
    src/udpsender.h
)

set(sources
    src/espwifistack.cpp
    src/espwifiutils.cpp
    src/udpsender.cpp
)

set(dependencies
    esp_common
    esp_netif
    lwip
    esp_wifi
    driver
    esp_eth

    cxx-ring-buffer
    cpputils
    espchrono
    espcpputils
    fmt
)

idf_component_register(
    INCLUDE_DIRS
        src
    SRCS
        ${headers}
        ${sources}
    REQUIRES
        ${dependencies}
)

set_property(TARGET ${COMPONENT_LIB} PROPERTY CXX_STANDARD 23)

target_compile_options(${COMPONENT_TARGET}
    PRIVATE
        -fstack-reuse=all
        -fstack-protector-all
        -Wno-unused-function
        -Wno-deprecated-declarations
        -Wno-missing-field-initializers
        -Wno-parentheses
)
