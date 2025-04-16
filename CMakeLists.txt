cmake_minimum_required(VERSION 3.10)
project(qtgl VERSION 0.0.1)
include(E:/vcpkg/scripts/buildsystems/vcpkg.cmake)
find_package(Qt5 REQUIRED Core Widgets)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(Eigen3 CONFIG REQUIRED)
find_package(OpenCV CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)

add_definitions(-w)

add_subdirectory(qtgl)
