idf_component_register(SRCS "main.c"
                            "device_control.c"
                            "iot_cli_cmd.c"
                            "iot_uart_cli.c"
                            "ota_util.c"
                            "caps_switch.c"
                            "caps_lock.c"
                            "caps_voltageMeasurement.c"
                            "caps_temperatureMeasurement.c"
                            "caps_relativeHumidityMeasurement.c"
                            "caps_firmwareUpdate.c"
                    EMBED_FILES "device_info.json"
                                "onboarding_config.json"
                                "public_key.pem"
                                "root.pem"
                    )

set(STDK_IOT_CORE_USE_DEFINED_CONFIG "y")

add_subdirectory($ENV{STDK_CORE_PATH} iotcore)
target_link_libraries(${COMPONENT_LIB} PUBLIC iotcore)
