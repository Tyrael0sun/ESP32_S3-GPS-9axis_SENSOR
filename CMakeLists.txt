# ESP32-S3 三合一设备项目 CMakeLists.txt

cmake_minimum_required(VERSION 3.16)

# 设置组件目录
set(EXTRA_COMPONENT_DIRS
    components
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32s3_bike_gps)
