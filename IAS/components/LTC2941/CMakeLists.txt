set(ltc2941_srcs "src/esp_ltc2941.c"
       )

idf_component_register(SRCS "${ltc2941_srcs}"
                    INCLUDE_DIRS include
                    REQUIRES driver)