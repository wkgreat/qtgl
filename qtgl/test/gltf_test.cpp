#include <cassert>
#include <iostream>
#include "../gltfmodel.hpp"

static void asset_test(qtgl::GLTFModel* model) {
  std::cout << "=== asset_test start" << std::endl;
  assert(model->getAsset().version == "2.0");
  assert(model->getAsset().generator == "Open Asset Import Library (assimp v5.2.cf7d3637)");
  assert(model->getAsset().copyright == "");
  assert(model->getAsset().minVersion == "");
  std::cout << ">>> asset_test success" << std::endl;
}

static void scene_test(qtgl::GLTFModel* model) {
  std::cout << "=== scene_test start" << std::endl;
  assert(model->getScene() == 0);
  std::cout << ">>> scene_test success" << std::endl;
}

static void scenes_test(qtgl::GLTFModel* model) {
  std::cout << "=== scenes_test start" << std::endl;
  std::vector<qtgl::GLTFScene>& scenes = model->getScenes();
  assert(scenes.size() == 1);
  assert(scenes[0].getNodes()[0] == 0);
  std::cout << ">>> scenes_test success" << std::endl;
}

static void primitive_test(qtgl::GLTFModel* model) {
  std::cout << "=== primitive_test start" << std::endl;
  std::vector<qtgl::GLPrimitive> primitives = model->getPrimitives();
  assert(primitives.size() == 1);
  qtgl::Vertices worldpos = primitives[0].getWorldPos();
  int nrow = worldpos.rows();
  std::cout << "world pos:" << std::endl;
  for (int i = 0; i < nrow; ++i) {
    std::cout << worldpos.row(i) << std::endl;
  }
  std::cout << "indices: " << std::endl;
  qtgl::Indices3 indices = primitives[0].getIndices();
  nrow = indices.rows();
  for (int i = 0; i < nrow; ++i) {
    std::cout << indices.row(i) << std::endl;
  }
  std::cout << ">>> primitive_test success" << std::endl;
}

int main() {
  std::cout << "gltf_test START." << std::endl;
  spdlog::set_level(spdlog::level::trace);

  // std::string path = "E:\\codes\\practice\\qtgl\\data\\DamagedHelmet\\DamagedHelmet.gltf";
  std::string path = "E:\\codes\\practice\\qtgl\\data\\Cube\\Cube.gltf";

  qtgl::GLTFModel model(path);

  // asset_test(&model);
  // scene_test(&model);
  // scenes_test(&model);
  primitive_test(&model);

  std::cout << "gltf_test FINISHED." << std::endl;

  return 0;
}