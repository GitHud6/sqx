idf_component_register(SRCS "main.c"
                            "../ble/ble_bus.c"
                            "../driver/ble_driver_registry.c"
                            "../config/ble_profile.c"
                            "../config/ble_config_store.c"
                            "../config/ble_config_manager.c"
                            "../driver/ble_device0_driver.c"
                            "../driver/generic_gatt_driver.c"
                            "../protocol/gateway_event.c"
                            "../protocol/generic_gatt_protocol.c"
                            "../protocol/ble_device0/ble_device0_protocol.c"
                            "../wifi/wifi_manager.c"
                            "../provisioning/ble_provisioning_service.c"
                            "../mqtt/mqtt_gateway.c"
                            "../network/network_manager.c"
                            "../ota/ota_manager.c"
                            "../service/gateway_service.c"
                       INCLUDE_DIRS "."
                                    "../ble"
                                    "../driver"
                                    "../config"
                                    "../protocol"
                                    "../protocol/ble_device0"
                                    "../wifi"
                                    "../provisioning"
                                    "../mqtt"
                                    "../network"
                                    "../ota"
                                    "../service"
                       REQUIRES bt
                                nvs_flash
                                json
                                esp_wifi
                                esp_netif
                                esp_event
                                freertos
                                 mqtt
                                 mbedtls
                                 esp_timer
                                 esp_hw_support
                                 app_update
                                 esp_http_client
                       EMBED_TXTFILES "../config/devices.json")
