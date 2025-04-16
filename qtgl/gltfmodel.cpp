#include "gltfmodel.hpp"

namespace qtgl {

void GLTFAsset::fromJson(GLTFModel* model, json assetJson) {
  this->model = model;
  this->version = assetJson["version"];
  if (assetJson.contains("generator")) {
    this->generator = assetJson["generator"];
  }
  if (assetJson.contains("copyright")) {
    this->copyright = assetJson["copyright"];
  }
  if (assetJson.contains("minVersion")) {
    this->minVersion = assetJson["minVersion"];
  }
}

void GLTFScene::fromJson(json& data) {
  std::vector<int> nodes = data["nodes"].get<std::vector<int>>();
  for (int n : nodes) {
    this->addNode(n);
  }
}

void GLTFNode::fromJson(json& data) {
  if (data.contains("name")) {
    std::string name = data["name"];
    this->setName(name);
  } else {
    std::string name = "";
    this->setName(name);
  }
  if (data.contains("matrix")) {
    std::vector<double> matrix = data["matrix"].get<std::vector<double>>();
    this->setMatrix(matrix);
  } else if (data.contains("translation") || data.contains("rotation") || data.contains("scale")) {
    std::vector<double> translation = {0, 0, 0};
    std::vector<double> rotation = {0, 0, 0, 1};
    std::vector<double> scale = {1, 1, 1};
    if (data.contains("translation")) {
      translation = data["translation"].get<std::vector<double>>();
    }
    if (data.contains("rotation")) {
      rotation = data["rotation"].get<std::vector<double>>();
    }
    if (data.contains("scale")) {
      scale = data["scale"].get<std::vector<double>>();
    }
    this->setTransformArrays(translation, rotation, scale);
  } else {
    std::vector<double> matrix = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    this->setMatrix(matrix);
  }
  if (data.contains("mesh")) {
    this->setMesh(data["mesh"]);
  } else {
    this->setMesh(-1);
  }
  if (data.contains("children")) {
    std::vector<int> children = data["children"].get<std::vector<int>>();
    this->setChildren(children);
  }
}

GLTFMesh::~GLTFMesh() {
  for (GLTFPrimitive* p : primitives) {
    delete p;
  }
}

GLTFMesh::GLTFMesh(const GLTFMesh& other) {
  this->model = other.model;
  this->name = other.name;
  this->weights = other.weights;
  for (const GLTFPrimitive* p : other.primitives) {
    GLTFPrimitive* np = new GLTFPrimitive(*p);
    this->primitives.push_back(np);
  }
}

GLTFMesh& GLTFMesh::operator=(const GLTFMesh& other) {
  if (this != &other) {
    this->model = other.model;
    this->name = other.name;
    this->weights = other.weights;
    for (const GLTFPrimitive* p : other.primitives) {
      GLTFPrimitive* np = new GLTFPrimitive(*p);
      this->primitives.push_back(np);
    }
  }
  return *this;
}

void GLTFMesh::fromJson(json& data) {
  if (data.contains("name")) {
    std::string name = data["name"];
    this->setName(name);
  } else {
    std::string name = "";
    this->setName(name);
  }
  if (data.contains("weights")) {
    std::vector<double> weights = data["weights"].get<std::vector<double>>();
    this->setWeights(weights);
  }
  for (json primitiveJson : data["primitives"]) {
    GLTFPrimitive* primitive = new GLTFPrimitive(this);
    if (primitiveJson.contains("attributes")) {
      std::map<std::string, int> attrmap =
          primitiveJson["attributes"].get<std::map<std::string, int>>();
      primitive->setAttributes(attrmap);
    }
    if (primitiveJson.contains("indices")) {
      primitive->setIndices(primitiveJson["indices"]);
    } else {
      primitive->setIndices(-1);
    }
    if (primitiveJson.contains("material")) {
      primitive->setMaterial(primitiveJson["material"]);
    } else {
      primitive->setMaterial(-1);
    }
    if (primitiveJson.contains("mode")) {
      primitive->setMode(primitiveJson["mode"]);
    } else {
      primitive->setMaterial(4);
    }
    this->addPrimitive(primitive);
  }
}

void GLTFMaterial::fromJson(json& data) {
  if (data.contains("name")) {
    std::string name = data["name"];
    this->setName(name);
  } else {
    std::string name = "";
    this->setName(name);
  }
  int index;
  int texCoord;
  json pbrJson = data["pbrMetallicRoughness"];

  json pbrBaseColorJson = pbrJson["baseColorTexture"];
  index = pbrBaseColorJson["index"];
  texCoord = -1;
  if (pbrBaseColorJson.contains("texCoord")) {
    texCoord = pbrBaseColorJson["texCoord"];
  }
  this->setPbrBaseColorTexture(index, texCoord);

  json pbrMetallicRoughnessJson = pbrJson["metallicRoughnessTexture"];
  index = pbrMetallicRoughnessJson["index"];
  texCoord = -1;
  if (pbrMetallicRoughnessJson.contains("texCoord")) {
    texCoord = pbrMetallicRoughnessJson["texCoord"];
  }
  this->setPbrMetallicRoughnessTexture(index, texCoord);

  json normalTextureJson = data["normalTexture"];
  index = normalTextureJson["index"];
  texCoord = -1;
  double scale = 1.0;
  if (normalTextureJson.contains("texCoord")) {
    texCoord = normalTextureJson["texCoord"];
  }
  if (normalTextureJson.contains("scale")) {
    scale = normalTextureJson["scale"];
  }
  this->setNormalTexture(index, texCoord, scale);

  json emissiveTextureJson = data["emissiveTexture"];
  index = emissiveTextureJson["index"];
  texCoord = -1;
  if (emissiveTextureJson.contains("texCoord")) {
    texCoord = emissiveTextureJson["texCoord"];
  }
  this->setEmissiveTexture(index, texCoord);

  json occlusionTextureJson = data["occlusionTexture"];
  index = occlusionTextureJson["index"];
  texCoord = -1;
  if (occlusionTextureJson.contains("texCoord")) {
    texCoord = occlusionTextureJson["texCoord"];
  }
  double strength = 1.0;
  if (occlusionTextureJson.contains("strength")) {
    strength = occlusionTextureJson["strength"];
  }
  this->setOcclusionTexture(index, texCoord, strength);

  std::vector<double> emissiveFactor = data["emissiveFactor"].get<std::vector<double>>();
  this->setEmissiveFactor(emissiveFactor[0], emissiveFactor[1], emissiveFactor[2]);
}

void GLTFImage::fromJson(json& data) {
  // TODO add other fields
  std::string uri = data["uri"];
  this->setUri(uri);
  // TODO read image
}

void GLTFAccessor::fromJson(json& data) {
  int bufferView = data.contains("bufferView") ? data["bufferView"] : -1;
  this->setBufferView(bufferView);
  int bufferOffset = data.contains("byteOffset") ? data["byteOffset"] : 0;
  this->setByteOffset(bufferOffset);
  int componentType = data.contains("componentType");
  this->setComponentType(componentType);
  bool normalized = data.contains("normalized") ? data["normalized"].get<bool>() : false;
  this->setNormalized(normalized);
  int count = data["count"];
  this->setCount(count);
  std::string type = data["type"];
  this->setType(type);
  std::vector<double> max =
      data.contains("max") ? data["max"].get<std::vector<double>>() : std::vector<double>{};
  this->setMax(max);
  std::vector<double> min =
      data.contains("min") ? data["min"].get<std::vector<double>>() : std::vector<double>{};
  this->setMin(min);
  // TODO sparse
  std::string name = data.contains("name") ? data["name"] : "";
  this->setName(name);
}

void GLTFBufferView::fromJson(json& data) {
  int buffer = data["buffer"];
  this->setBuffer(buffer);
  int byteOffset = data.contains("byteOffset") ? data["byteOffset"] : 0;
  int byteLength = data["byteLength"];
  this->setByteLength(byteLength);
  int byteStride = data.contains("byteStride") ? data["byteStride"] : -1;
  this->setByteStride(byteStride);
  int target = data.contains("target") ? data["target"] : -1;
  this->setTarget(target);
  std::string name = data.contains("name") ? data["name"] : "";
  this->setName(name);
}

void GLTFBuffer::fromJson(json& data) {
  std::string uri = data.contains("uri") ? data["uri"] : "";
  this->setUri(uri);
  int byteLength = data["byteLength"];
  this->setByteLength(byteLength);
  std::string name = data.contains("name") ? data["name"] : "";
  this->setName(name);
}

void GLTFModel::parseFromFile(std::string& path) {
  std::ifstream f(path);
  json data = json::parse(f);
  // asset
  json assetJson = data["asset"];
  asset.fromJson(this, assetJson);
  f.close();
  // scene
  this->scene = data["scene"];
  // scenes
  for (json sceneJson : data["scenes"]) {
    GLTFScene scene(this);
    scene.fromJson(sceneJson);
    this->addScene(scene);
  }
  // nodes
  for (json nodeJson : data["nodes"]) {
    GLTFNode node(this);
    node.fromJson(nodeJson);
    this->addNode(node);
  }
  // meshes
  for (json meshJson : data["meshes"]) {
    GLTFMesh mesh(this);
    mesh.fromJson(meshJson);
    this->addMesh(mesh);
  }
  // material
  for (json materialJson : data["materials"]) {
    GLTFMaterial material(this);
    material.fromJson(materialJson);
    this->addMaterial(material);
  }
  // images
  for (json imageJson : data["images"]) {
    GLTFImage image(this);
    image.fromJson(imageJson);
    this->addImage(image);
  }
  // accessors
  for (json accessorJson : data["accessors"]) {
    GLTFAccessor accessor(this);
    accessor.fromJson(accessorJson);
    this->addAccessor(accessor);
  }
  // bufferViews
  for (json bufferViewJson : data["bufferViews"]) {
    GLTFBufferView bufferView(this);
    bufferView.fromJson(bufferViewJson);
    this->addBufferView(bufferView);
  }
  // buffers
  for (json bufferJson : data["buffers"]) {
    GLTFBuffer buffer(this);
    buffer.fromJson(bufferJson);
    this->addBuffer(buffer);
  }
}

};  // namespace qtgl