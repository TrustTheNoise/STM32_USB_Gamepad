add_library(config INTERFACE)

target_include_directories(config INTERFACE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(config INTERFACE mcu)

add_subdirectory(HAL_configuration)
add_subdirectory(USB_configuration)

target_link_libraries(config INTERFACE hal_configuration)
target_link_libraries(config INTERFACE usb_configuration)

if(TARGET stm32_sdk)
    target_link_libraries(config INTERFACE stm32_sdk)
    target_compile_definitions(config INTERFACE USE_STM32_SDK)
endif()