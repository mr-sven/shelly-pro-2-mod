idf_component_register(
    SRCS
        main.c
        wifi.c
        ota.c
        mqtt.c
        shift_reg.c
        ds18b20.c
    INCLUDE_DIRS "."
    REQUIRES
        esp_wifi
        esp_adc
        app_update
        esp_http_client
        nvs_flash
        mqtt
        driver
        onewire_bus)
