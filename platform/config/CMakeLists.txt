add_library(config INTERFACE)

target_include_directories(config INTERFACE ./)
target_link_libraries(config INTERFACE mcu)

if(TARGET stm32_sdk)
    target_link_libraries(config INTERFACE stm32_sdk)
    target_compile_definitions(config INTERFACE USE_STM32_SDK)
endif()