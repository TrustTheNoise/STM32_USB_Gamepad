add_library(driver STATIC 
    usb_device_driver.c
)

target_include_directories(driver PUBLIC ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(driver PUBLIC config)