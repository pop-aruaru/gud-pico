# SPDX-License-Identifier: MIT
if(NOT PICO_PLATFORM STREQUAL "rp2040")
    add_subdirectory(ili9488)
    add_subdirectory(st7796)
endif()
add_subdirectory(ili9341)
