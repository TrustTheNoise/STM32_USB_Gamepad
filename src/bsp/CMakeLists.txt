add_library(bsp STATIC 
    adc_bsp.c
    buttons_bsp.c
)

target_include_directories(bsp PUBLIC ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(bsp PUBLIC config)