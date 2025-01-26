add_executable(ili9488
  ili9488.c
)

target_include_directories(ili9488 PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(ili9488 PRIVATE pico_stdlib gud_pico mipi_dbi hardware_pwm pico_unique_id)
pico_add_extra_outputs(ili9488)
