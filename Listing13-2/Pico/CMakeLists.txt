cmake_minimum_required(VERSION 3.16)
include(pico_sdk_import.cmake)
project(Listing13-2 C CXX ASM)
pico_sdk_init()

add_executable(Listing13-2
    main.cpp
)

# pull in common dependencies and additional i2c hardware support
target_link_libraries(Listing13-2 pico_stdlib pico_multicore hardware_i2c)

# create map/bin/hex file etc.
pico_add_extra_outputs(Listing13-2)

# stdout destination is USB CDC
pico_enable_stdio_usb(Listing13-2 1)

