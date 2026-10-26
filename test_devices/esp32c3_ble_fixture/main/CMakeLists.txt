idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES bt
                                esp_timer)

set(NETGATE_C3_FIXTURE_VARIANT 0 CACHE STRING
    "BLE fixture profile: 0=Ray7, 1=Netgate-C3-Device1")
target_compile_definitions(${COMPONENT_LIB} PRIVATE
                            NETGATE_C3_FIXTURE_VARIANT=${NETGATE_C3_FIXTURE_VARIANT})
