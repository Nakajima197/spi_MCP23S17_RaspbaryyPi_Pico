add_executable(spi_MCP23S17
        spi_MCP23S17.c
        )

# Pull in basic dependencies
target_link_libraries(spi_MCP23S17 pico_stdlib hardware_spi pico_rand hardware_pwm)

# create map/bin/hex file etc.
pico_add_extra_outputs(spi_MCP23S17)

# add url via pico_set_program_url
example_auto_set_url(spi_MCP23S17)

# enable usb output, disable uart output
pico_enable_stdio_usb(spi_MCP23S17 1)
pico_enable_stdio_uart(spi_MCP23S17 0)
