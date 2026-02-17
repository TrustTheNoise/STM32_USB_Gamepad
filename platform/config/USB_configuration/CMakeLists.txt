add_library(usb_configuration STATIC 
    usb_descriptors.c
    ${CMAKE_CURRENT_LIST_DIR}/bsp/board.c
)

target_include_directories(usb_configuration PUBLIC 
    ./
    ${CMAKE_CURRENT_LIST_DIR}/bsp
)

tinyusb_target_add(usb_configuration)
target_link_libraries(usb_configuration PUBLIC config)
target_compile_options(usb_configuration PUBLIC -g3 -ggdb)