set(MACHINE_PROFILE_JSON "${CMAKE_CURRENT_LIST_DIR}/../config/machine_profile.json")
set(MACHINE_PROFILE_GENERATOR "${CMAKE_CURRENT_LIST_DIR}/../cmake/generate_machine_profile.py")
set(MACHINE_PROFILE_GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(MACHINE_PROFILE_GENERATED_HEADER "${MACHINE_PROFILE_GENERATED_DIR}/machine_profile_generated.hpp")
file(MAKE_DIRECTORY "${MACHINE_PROFILE_GENERATED_DIR}")

idf_component_register(SRCS "app_main.cpp"
                            "../command/command_executor.cpp"
                            "../driver/rmt_step_pulse_driver.cpp"
                            "../driver/limit_switch_driver.cpp"
                            "../driver/step_timer_driver.cpp"
                            "../driver/uart0_driver.cpp"
                            "../driver/sdcard_driver.cpp"
                            "../driver/lis2dh12_driver.cpp"
                            "../accelerometer/accelerometer.cpp"
                            "../homing/homing_service.cpp"
                            "../laser/laser_driver.cpp"
                            "../laser/laser_planner_adapter.cpp"
                            "../machine/machine_profile.cpp"
                            "../motion/motion_submit.cpp"
                            "../network/network_command_adapter.cpp"
                            "../network/network_lifecycle.cpp"
                            "../planner/planner.cpp"
                            "../protocol/command_dispatch.cpp"
                            "../protocol/line_assembler.cpp"
                            "../protocol/protocol_task.cpp"
                            "../protocol/protocol_tx.cpp"
                            "../protocol/realtime_command.cpp"
                            "../sd/sd_command_handler.cpp"
                            "../sd/plain_file_reader.cpp"
                            "../sd/sd_file_service.cpp"
                            "../sd/sd_job_service.cpp"
                            "../service/service.cpp"
                            "../httpd/httpd_service.cpp"
                            "../websocket/websocket_service.cpp"
                            "../wifi/wifi_service.cpp"
                            "../wifi/wifi_settings.cpp"
                            "../stepper/stepper_driver.cpp"
                            "../stepper/motion_profile.cpp"
                            "../stepper/trapezoid_execution_plan.cpp"
                            "../stepper/trapezoid_segment_generator.cpp"
                            "../stepper/segment_ring.cpp"
                            "../stepper/segment_producer.cpp"
                            "../stepper/stepper_prep.cpp"
                            "../stepper/timer_stepper_executor.cpp"
                       INCLUDE_DIRS "."
                                    "../command"
                                    "../driver"
                                    "../accelerometer"
                                    "../homing"
                                    "../identity"
                                    "../laser"
                                    "../machine"
                                    "${MACHINE_PROFILE_GENERATED_DIR}"
                                    "../motion"
                                    "../network"
                                    "../planner"
                                    "../protocol"
                                    "../sd"
                                    "../service"
                                    "../httpd"
                                    "../websocket"
                                    "../wifi"
                                    "../stepper"
                       REQUIRES esp_driver_gpio
                                driver
                                esp_driver_ledc
                                esp_driver_rmt
                                esp_driver_uart
                                esp_driver_i2c
                                fatfs
                                sdmmc
                                esp_rom
                                esp_wifi
                                esp_event
                                esp_netif
                                esp_http_server
                                esp_timer
                                log
                                mbedtls
                                nvs_flash
                                lwip)

add_custom_command(
    OUTPUT "${MACHINE_PROFILE_GENERATED_HEADER}"
    COMMAND "${CMAKE_COMMAND}" -E make_directory "${MACHINE_PROFILE_GENERATED_DIR}"
    COMMAND "${PYTHON}" "${MACHINE_PROFILE_GENERATOR}"
            --input "${MACHINE_PROFILE_JSON}"
            --output "${MACHINE_PROFILE_GENERATED_HEADER}"
    DEPENDS "${MACHINE_PROFILE_JSON}" "${MACHINE_PROFILE_GENERATOR}"
    VERBATIM)
add_custom_target(machine_profile_generated DEPENDS "${MACHINE_PROFILE_GENERATED_HEADER}")
add_dependencies(${COMPONENT_LIB} machine_profile_generated)
