idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES esp32-ism330dlc
                                  esp_driver_gpio
                                  esp_driver_i2c)
