# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

idf_component_register(SRCS "main.c" "epd_driver.c"
                       INCLUDE_DIRS "." "../include"
                       REQUIRES driver esp_timer esp_rom heap freertos log)
