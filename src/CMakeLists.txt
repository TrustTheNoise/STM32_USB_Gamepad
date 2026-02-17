add_library(src STATIC main.c)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/bsp)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/driver)

tinyusb_target_add(src)
target_link_libraries(src PUBLIC config)
target_link_libraries(src PUBLIC bsp)
target_link_libraries(src PUBLIC driver)