add_library(mcu INTERFACE)

target_include_directories(mcu INTERFACE 
    ${CMAKE_CURRENT_LIST_DIR}
    CMSIS_Core
)

target_link_libraries(mcu INTERFACE stm32f1_hal)