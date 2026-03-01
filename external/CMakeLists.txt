# adding external cmake libraries
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/STM32_CMSIS_SDK)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/TinyUSB)

# STM32 HAL isn't cmake library so we should made library by own
set(STM32_HAL_DIR ${CMAKE_CURRENT_LIST_DIR}/stm32f1xx-hal-driver)

add_library(stm32f1_hal STATIC)

file(GLOB_RECURSE stm32f1_hal_src
    ${STM32_HAL_DIR}/Src/*.c
)

target_sources(stm32f1_hal PRIVATE ${stm32f1_hal_src})

target_link_libraries(stm32f1_hal PRIVATE config)

target_include_directories(stm32f1_hal PUBLIC 
    ${STM32_HAL_DIR}/Inc/
)