# The following four lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
set(PROJECT_VER "0.4.2")  # 👈 your custom version here
project(esp32-VitalX VERSION ${PROJECT_VER})

target_add_binary_data(esp32-VitalX.elf "main/isrgrootx1.pem" TEXT)