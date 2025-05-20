idf_component_register(SRCS "main.c"
                    PRIV_REQUIRES spi_flash
                    REQUIRES esp_adc
                    REQUIRES driver
                    REQUIRES esp_driver_ledc
                    REQUIRES esp_driver_gptimer
                    INCLUDE_DIRS "")
