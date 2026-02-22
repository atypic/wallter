idf_component_register(
    SRCS "factory_test.cpp" "main.cpp" "motordriver_cytron.cpp" "display_rgb.cpp" "buttons.cpp" "calibration_store.cpp" "modes.cpp" "motor_control.cpp" "inputs.cpp"
    INCLUDE_DIRS "."
    REQUIRES espidf_lcd cytron_md pid driver esp_driver_gpio nvs_flash
)