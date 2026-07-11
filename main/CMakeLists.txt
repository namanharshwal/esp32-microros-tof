idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        driver
        esp_wifi
        esp_netif
        nvs_flash
        freertos
        esp_timer
        micro_ros_espidf_component
)
