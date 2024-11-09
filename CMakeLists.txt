if (NOT TARGET st7735_80x160)

    # Driver initialize (STATIC or INTERFACE, second basically contains only .h headers)
    add_library(st7735_80x160 STATIC)

    # Including .c files (PRIVATE - only library can access code files)
    target_sources(st7735_80x160 PRIVATE
        ${LIBRARIES_DIR}/st7735_80x160/src/st7735.cpp
    )
    
    # Including .h header files (PUBLIC - user can use driver only through 'friendly' header)
    target_include_directories(st7735_80x160 PUBLIC
        ${LIBRARIES_DIR}/st7735_80x160/include
    )

    # Including additional libraries (PRIVATE - these libraries don't read this one)
    target_link_libraries(st7735_80x160 PRIVATE
        hardware_spi
        hardware_clocks
        hardware_pwm

        pico_stdlib
    )

endif() # TARGET st7735_80x160
