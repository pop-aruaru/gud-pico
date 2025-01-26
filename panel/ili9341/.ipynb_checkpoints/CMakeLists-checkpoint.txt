add_executable(ili9341
  ili9341.c
)

target_include_directories(ili9341 PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(ili9341 PRIVATE pico_stdlib gud_pico mipi_dbi hardware_pwm pico_unique_id)
pico_add_extra_outputs(ili9341)
