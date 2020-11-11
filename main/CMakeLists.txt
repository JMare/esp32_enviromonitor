cmake_minimum_required(VERSION 3.5)
set(srcs "main.c" "ui.c")

idf_component_register(SRCS "${srcs}"
                            REQUIRES tft spidriver esp-idf-bme280)
target_compile_options(${COMPONENT_LIB} PRIVATE )
