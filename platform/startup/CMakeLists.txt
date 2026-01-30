add_library(startup STATIC
    startup_stm32f103xx.s
    syscall.c
    sysmem.c
)