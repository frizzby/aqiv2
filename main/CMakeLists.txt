set (CMAKE_CXX_STANDARD 17)
idf_component_register(SRCS "shared.cpp" "cmdline.cpp" "hpma115s0.cpp" "hpma115S0.cpp" "hpma.cpp" "main.cpp"
                       "u8g2_esp32_hal.cpp"
                       "dispatcher.cpp"
                       "display.cpp"
                       "wifi.cpp"
                       "udp.cpp"
                       "bme280.cpp"
                       "hpma.cpp"
                       "hpma115S0.cpp"
                       "cmdline.cpp"
                       "shared.cpp"
                    INCLUDE_DIRS "." )
