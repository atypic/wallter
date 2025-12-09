idf_component_register(
    SRCS "rgb_lcd_idf.cpp" "demo.cpp"
    INCLUDE_DIRS "."
    REQUIRES driver freertos
)
