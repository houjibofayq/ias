set(lora_srcs "src/lora.c"
       )

idf_component_register(SRCS "${lora_srcs}"
                    INCLUDE_DIRS include
                    REQUIRES driver)