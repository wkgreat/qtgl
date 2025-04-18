set(CMAKE_INCLUDE_CURRENT_DIR true)
add_executable(qtglmain event.cpp texture.cpp gltfmodel.cpp objmodel.cpp scene.cpp shadow.cpp shader.cpp mesh.cpp primitive.cpp qtglmain.cpp)
target_link_libraries(qtglmain spdlog::spdlog Qt5::Core Qt5::Widgets Eigen3::Eigen ${OpenCV_LIBS} nlohmann_json::nlohmann_json)
add_subdirectory(test)