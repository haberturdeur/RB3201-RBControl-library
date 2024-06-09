cmake_minimum_required(VERSION 3.16)

FILE(GLOB_RECURSE app_sources src/*.cpp src/*.c)

idf_component_register(
    SRCS ${app_sources}
    INCLUDE_DIRS "src"
    REQUIRES driver nvs_flash esp_timer esp_adc esp_hw_support lx16a I2C
)
