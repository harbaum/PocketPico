add_library(ili9xxx_lcd)

target_sources(ili9xxx_lcd
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/ili9xxx_lcd.c
        ${CMAKE_CURRENT_SOURCE_DIR}/ili9xxx_font.c
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/ili9xxx_lcd.h
        ${CMAKE_CURRENT_SOURCE_DIR}/ili9xxx_font.c
)

target_compile_definitions(ili9xxx_lcd PRIVATE
	ILI9341=1
)

pico_generate_pio_header(ili9xxx_lcd ${CMAKE_CURRENT_LIST_DIR}/ili9225_lcd.pio)

target_include_directories(ili9xxx_lcd PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ili9xxx_lcd PUBLIC pico_stdlib hardware_pio hardware_dma hardware_spi)
