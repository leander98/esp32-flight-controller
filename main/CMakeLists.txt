idf_component_register(SRCS "main.c" "flight-controller.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES esp32-ism330dlc
                                  esp32-esc-xw30a
                                  esp32-wifi-drone-remote
                                  esp_driver_gpio
                                  esp_driver_i2c
                                  esp_timer
                                  nvs_flash)
