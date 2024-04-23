add_library(comms STATIC
)

target_link_libraries(comms
  pico_stdlib
  pico_stdio_usb
  pico_multicore
  hardware_pwm
  hardware_pio
)

target_include_directories(comms PUBLIC
  include
)