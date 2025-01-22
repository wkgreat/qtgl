set(CMAKE_INCLUDE_CURRENT_DIR true)
add_executable(qtglmain texture.cpp objmodel.cpp mesh.cpp scene.cpp qtglmain.cpp)
target_link_libraries(qtglmain Qt5::Core Qt5::Widgets Eigen3::Eigen ${OpenCV_LIBS})

add_subdirectory(test)