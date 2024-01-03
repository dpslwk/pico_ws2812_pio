pico_add_library(pico_ws2812_pio)

pico_generate_pio_header(pico_ws2812_pio
        ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio)

target_sources(pico_ws2812_pio INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/pico_ws2812_pio.c
        )

target_include_directories(pico_ws2812_pio INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/include)

pico_mirrored_target_link_libraries(pico_ws2812_pio INTERFACE
        pico_stdlib
        hardware_pio
        hardware_irq
        hardware_dma
        pico_async_context_freertos
        )
