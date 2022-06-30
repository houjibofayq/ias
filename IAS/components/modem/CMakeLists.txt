set(modem_srcs "src/esp_modem.c"
        "src/esp_modem_dce_service"
        "src/esp_modem_netif.c"
        "src/esp_modem_compat.c"
        "src/esp_modem_sim800.c"
        "src/esp_modem_bg96.c")

idf_component_register(SRCS "${modem_srcs}"
                    INCLUDE_DIRS include
                    REQUIRES driver)