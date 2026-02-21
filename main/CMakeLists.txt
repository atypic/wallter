idf_component_register(
    SRCS "main.cpp" "motordriver_cytron.cpp" "display_rgb.cpp" "buttons.cpp" "calibration_store.cpp"
    INCLUDE_DIRS "."
    REQUIRES espidf_lcd cytron_md pid driver esp_driver_gpio nvs_flash
)