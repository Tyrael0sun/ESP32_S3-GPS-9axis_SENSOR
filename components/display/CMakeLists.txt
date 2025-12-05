idf_component_register(
    SRCS "display_driver.cpp"
    INCLUDE_DIRS "include"
    REQUIRES lvgl LovyanGFX common
)

# LovyanGFX 依赖 C++17 特性，强制启用
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)
