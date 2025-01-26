add_executable(st7796m
  st7796m.c
)

target_include_directories(st7796m PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(st7796m PRIVATE pico_stdlib gud_pico mipi_dbi hardware_pwm pico_unique_id)
pico_add_extra_outputs(st7796m)
