set(ina219_srcs "src/esp_ina219.c"
       )

idf_component_register(SRCS "${ina219_srcs}"
                    INCLUDE_DIRS include
                    REQUIRES driver)