set(CMAKE_INCLUDE_CURRENT_DIR true)
include_directories(${CMAKE_SOURCE_DIR}/..)
add_executable(scene_test ../objmodel.cpp ../mesh.cpp scene_test.cpp)
target_link_libraries(scene_test Qt5::Core Qt5::Widgets Eigen3::Eigen ${OpenCV_LIBS})