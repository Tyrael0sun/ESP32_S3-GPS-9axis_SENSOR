idf_component_register(
    SRCS "display_driver.cpp"
    INCLUDE_DIRS "include"
    REQUIRES lvgl LovyanGFX common
)

# LovyanGFX 依赖 C++17 特性，强制启用
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)

# 自动修补 LovyanGFX 在 C23 下的 bool 重新定义问题
idf_component_get_property(LOVYANGFX_DIR COMPONENT_DIR lovyan03__LovyanGFX)
if(LOVYANGFX_DIR)
    set(LOVYANGFX_QRCODE "${LOVYANGFX_DIR}/src/lgfx/utility/lgfx_qrcode.h")
    if(EXISTS ${LOVYANGFX_QRCODE})
        file(READ ${LOVYANGFX_QRCODE} QRCODE_CONTENT)
        string(FIND "${QRCODE_CONTENT}" "#ifndef __cplusplus" PATCHED_POS)
        if(PATCHED_POS EQUAL -1)
            string(REPLACE "typedef unsigned char bool;\nstatic const bool false = 0;\nstatic const bool true = 1;\n"
                           "#ifndef __cplusplus\n#include <stdbool.h>\n#endif\n"
                           QRCODE_CONTENT "${QRCODE_CONTENT}")
            file(WRITE ${LOVYANGFX_QRCODE} "${QRCODE_CONTENT}")
        endif()
    endif()
endif()
