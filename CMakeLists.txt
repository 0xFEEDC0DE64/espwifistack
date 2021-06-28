set(headers
    src/espwifistack.h
    src/espwifistackconfig.h
    src/espwifistackenums.h
    src/espwifiutils.h
)

set(sources
    src/espwifistack.cpp
    src/espwifistackenums.cpp
    src/espwifiutils.cpp
)

set(dependencies
    cpputils
    espchrono
    espcpputils
    expected
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

target_compile_options(${COMPONENT_TARGET}
    PRIVATE
        -fstack-reuse=all
        -fstack-protector-all
        -Wno-unused-function
        -Wno-deprecated-declarations
        -Wno-missing-field-initializers
        -Wno-parentheses
)