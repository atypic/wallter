idf_component_register(
    SRCS "main.c" "motordriver_cytron.cpp" "display_rgb.cpp"
    INCLUDE_DIRS "."
    REQUIRES espidf_lcd cytron_md pid
)