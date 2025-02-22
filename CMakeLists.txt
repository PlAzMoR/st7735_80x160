if (NOT TARGET st7735_80x160)

    # Driver initialize (STATIC or INTERFACE, second basically contains only .h headers)
    add_library(st7735_80x160 STATIC)

    # Including .c/.cpp files (PRIVATE - only library can access code files)
    target_sources(st7735_80x160 PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/src/st7735.cpp
    )
    
    # Including .h header files (PUBLIC - user can use driver only through 'friendly' header including)
    target_include_directories(st7735_80x160 PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/include
    )

    # Including required public libraries
    # (PUBLIC - these libs imports to project by main header including)
    target_link_libraries(st7735_80x160 PUBLIC
        hardware_spi
        hardware_clocks
        hardware_pwm
    )

    # Including private libraries 
    # (PRIVATE - these libs you need to import from project)
    target_link_libraries(st7735_80x160 PRIVATE
        pico_stdlib
    )

endif() # TARGET st7735_80x160