set(uart_srcs
    "src/esp_uart_config.c")

idf_component_register(SRCS "${uart_srcs}"
                       INCLUDE_DIRS include
                    REQUIRES driver)
