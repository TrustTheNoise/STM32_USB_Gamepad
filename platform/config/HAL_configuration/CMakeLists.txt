add_library(hal_configuration STATIC
    system_stm32f1xx.c
)

target_include_directories(hal_configuration PUBLIC
    ./
)

target_link_libraries(hal_configuration PRIVATE mcu)
target_link_libraries(hal_configuration PUBLIC stm32f1_hal)

target_compile_options(hal_configuration PUBLIC -g3 -ggdb)