set(max1704_srcs "src/esp_max1704.c"
       )

idf_component_register(SRCS "${max1704_srcs}"
                    INCLUDE_DIRS include
                    REQUIRES driver)