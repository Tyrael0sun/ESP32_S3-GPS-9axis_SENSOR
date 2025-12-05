idf_component_register(
    SRCS "ui_manager.c" "ui_main_menu.c" "ui_bike_page.c" "ui_gpx_page.c" "ui_pgear_page.c" "ui_settings_page.c"
    INCLUDE_DIRS "include"
    REQUIRES lvgl data_model
)
