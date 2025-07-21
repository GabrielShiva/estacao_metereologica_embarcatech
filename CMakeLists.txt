# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================

cmake_minimum_required(VERSION 3.13)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(PICO_BOARD pico_w CACHE STRING "Board type")
include(pico_sdk_import.cmake)

project(estacao_metereologica_web C CXX ASM)

pico_sdk_init()

include_directories(
    ${CMAKE_SOURCE_DIR}/inc
    ${CMAKE_SOURCE_DIR}/src
)

add_executable(estacao_metereologica_web
    src/main.c
    inc/sensors/aht20.c
    inc/sensors/bmp280.c
    inc/display/ssd1306.c
    inc/button/button.c
    inc/buzzer/buzzer.c
    inc/led_matrix/ws2812b.c
    inc/led_rgb/led.c
)

pico_set_program_name(estacao_metereologica_web "estacao_metereologica_web")
pico_set_program_version(estacao_metereologica_web "0.1")

pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/inc/led_matrix/ws2812b.pio)

pico_enable_stdio_uart(estacao_metereologica_web 0)
pico_enable_stdio_usb(estacao_metereologica_web 1)

target_link_libraries(estacao_metereologica_web
    pico_stdlib
    hardware_i2c
    hardware_pio
    pico_cyw43_arch_lwip_threadsafe_background
    hardware_adc
    hardware_pwm
    hardware_clocks
)

target_include_directories(estacao_metereologica_web PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/inc
        ${CMAKE_CURRENT_LIST_DIR}/src
        ${PICO_SDK_PATH}/inc/lwip/src/include
        ${PICO_SDK_PATH}/inc/lwip/src/include/arch
        ${PICO_SDK_PATH}/inc/lwip/src/include/lwip
)

target_sources(${PROJECT_NAME} PRIVATE
    ${PICO_SDK_PATH}/inc/lwip/src/apps/http/httpd.c
    ${PICO_SDK_PATH}/inc/lwip/src/apps/http/fs.c
)

pico_add_extra_outputs(estacao_metereologica_web)
