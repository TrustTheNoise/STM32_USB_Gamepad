add_library(mcu INTERFACE)

target_include_directories(mcu INTERFACE ./)
target_include_directories(mcu INTERFACE CMSIS_Core)