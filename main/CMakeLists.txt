idf_component_register(
    SRCS "app_main.c"
    INCLUDE_DIRS ""
    REQUIRES common data_model sensor gnss input power logger gpx bike pgear ui display
)
