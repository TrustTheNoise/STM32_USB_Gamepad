add_library(src STATIC main.c)

target_link_libraries(src PRIVATE config)