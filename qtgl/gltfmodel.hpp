#pragma once

#include <spdlog/spdlog.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>
#include "model.hpp"

using json = nlohmann::json;

namespace qtgl {

class GLTFModel;
class GLTFAsset;
class GLTFScene;
class GLTFNode;
class GLTFMesh;
class GLTFPrimitive;
class GLTFMaterial;
class GLTFImage;
class GLTFAccessor;
class GLTFBufferView;
class GLTFBuffer;

class GLTFAsset {
 public:
  GLTFModel* model = nullptr;
  std::string version = "";
  std::string generator = "";
  std::string copyright = "";
  std::string minVersion = "";
  GLTFAsset() = default;
  void fromJson(GLTFModel* model, json assetJson);
};

class GLTFScene {
 private:
  GLTFModel* model;
  std::vector<int> nodes;

 public:
  GLTFScene(GLTFModel* model) : model(model) {}
  ~GLTFScene() = default;
  void addNode(int node) { nodes.push_back(node); }
  void fromJson(json& data);
  std::vector<int>& getNodes() { return this->nodes; }
};

class GLTFNode {
 private:
  GLTFModel* model;
  std::string name;
  bool hasTransform = false;
  bool isTransformSeperate = false;
  union transform {
    double matrix[16];
    struct {
      double translation[3];
      double rotation[4];  // quaternion
      double scale[3];
    } arrays;
  } transform;
  int mesh;
  std::vector<int> children;

 public:
  GLTFNode(GLTFModel* model) : model(model) {}
  void setName(std::string& name) { this->name = name; }
  void setMatrix(std::vector<double>& matrix) {
    hasTransform = true;
    isTransformSeperate = false;
    for (int i = 0; i < 16; ++i) {
      this->transform.matrix[i] = matrix[i];
    }
  }
  void setTransformArrays(std::vector<double>& translation, std::vector<double>& rotation,
                          std::vector<double>& scale) {
    hasTransform = true;
    isTransformSeperate = true;
    this->transform.arrays.translation[0] = translation[0];
    this->transform.arrays.translation[1] = translation[1];
    this->transform.arrays.translation[2] = translation[2];
    this->transform.arrays.rotation[0] = rotation[0];
    this->transform.arrays.rotation[1] = rotation[1];
    this->transform.arrays.rotation[2] = rotation[2];
    this->transform.arrays.rotation[3] = rotation[3];
    this->transform.arrays.scale[0] = scale[0];
    this->transform.arrays.scale[1] = scale[1];
    this->transform.arrays.scale[2] = scale[2];
  }
  void setMesh(int mesh) { this->mesh = mesh; }
  void setChildren(std::vector<int>& childern) { this->children = childern; }
  GLTFModel* getModel() { return this->model; }
  std::string getName() { return this->name; }
  union transform& getTransform() { return transform; }
  bool getHasTransform() const { return hasTransform; }
  bool getIsTransformSep() const { return isTransformSeperate; }
  std::vector<int>& getChildren() { return children; }
  int getMesh() { return this->mesh; }

  void fromJson(json& data);
};

class GLTFMesh {
 private:
  GLTFModel* model;
  std::string name;
  std::vector<double> weights;
  std::vector<GLTFPrimitive*> primitives;

 public:
  GLTFMesh(GLTFModel* model) : model(model) {}
  ~GLTFMesh();
  GLTFMesh(const GLTFMesh& other);
  GLTFMesh& operator=(const GLTFMesh& other);
  GLTFModel* getModel() const { return this->model; }
  void setName(std::string& name) { this->name = name; }
  void addPrimitive(GLTFPrimitive* primitive) { this->primitives.push_back(primitive); }
  void setWeights(std::vector<double>& weights) { this->weights = weights; }
  std::string getName() { return this->name; }
  std::vector<GLTFPrimitive*> getPrimitives() { return this->primitives; }
  void fromJson(json& data);
};

class GLTFPrimitive {
 private:
  GLTFMesh* mesh;
  int mode;
  int material;
  int indices;
  std::map<std::string, int> attributes;
  // TODO add morph targets

 public:
  GLTFPrimitive(GLTFMesh* mesh) : mesh(mesh) {}
  ~GLTFPrimitive() = default;

  void setMode(int mode) { this->mode = mode; }
  void setMaterial(int material) { this->material = material; }
  void setIndices(int indices) { this->indices = indices; }
  void setAttributes(std::map<std::string, int>& attrs) { this->attributes = attrs; }
  int getMode() const { return this->mode; }
  GLTFMesh* getMesh() const { return this->mesh; }
  int getMaterial() const { return this->material; }
  int getIndices() const { return this->indices; }
  int getAttribute(std::string k) { return this->attributes[k]; }
};

class GLTFMaterial : public GLMaterialBase {
 private:
  GLTFModel* model;
  std::string name;
  struct pbrMetallicRoughness {
    // TODO baseColorFactor metallicFactor roughnessFactor
    struct {
      int index = -1;
      int texCoord = 0;
    } baseColorTexture;
    double baseColorFactor[4] = {1, 1, 1, 1};
    struct {
      int index = -1;
      int texCoord = 0;
    } metallicRoughnessTexture;
    double metallicFactor = 1.0;
    double roughnessFactor = 1.0;
  } pbrMetallicRoughness;
  struct normalTexture {
    double scale = 1.0;
    int index = -1;
    int texCoord = 0;
  } normalTexture;
  struct emissiveTexture {
    int index = -1;
    int texCoord = 0;
  } emissiveTexture;
  struct occlusionTexture {
    double strength = 1.0;
    int index = -1;
    int texCoord = 0;
  } occlusionTexture;
  double emissiveFactor[3];

  bool hasPbrBaseColorTexture = false;
  bool hasPbrMetallicRoughnessTexture = false;
  bool hasNormalTexture = false;
  bool hasEmissiveTexture = false;
  bool hasOcclusionTexture = false;

 public:
  GLTFMaterial(GLTFModel* model) : model(model) { this->illumination = IlluminationModel::PBR; }
  void setName(std::string& name) { this->name = name; }
  void setPbrBaseColorTexture(int index, int texCoord = -1) {
    this->pbrMetallicRoughness.baseColorTexture.index = index;
    this->pbrMetallicRoughness.baseColorTexture.texCoord = texCoord;
  }
  void setPbrBaseColorFactor(std::vector<double>& factor) {
    this->pbrMetallicRoughness.baseColorFactor[0] = factor[0];
    this->pbrMetallicRoughness.baseColorFactor[1] = factor[1];
    this->pbrMetallicRoughness.baseColorFactor[2] = factor[2];
  }
  void setPbrMetallicRoughnessTexture(int index, int texCoord = -1) {
    this->pbrMetallicRoughness.metallicRoughnessTexture.index = index;
    this->pbrMetallicRoughness.metallicRoughnessTexture.texCoord = texCoord;
  }
  void setPbrMetallicFactor(double metallic) {
    this->pbrMetallicRoughness.metallicFactor = metallic;
  }
  void setPbrRoughnessFactor(double roughness) {
    this->pbrMetallicRoughness.roughnessFactor = roughness;
  }
  void setNormalTexture(int index, int texCoord = -1, double scale = 1.0) {
    this->normalTexture.index = index;
    this->normalTexture.texCoord = texCoord;
    this->normalTexture.scale = scale;
  }
  void setEmissiveTexture(int index, int texCoord = -1) {
    this->emissiveTexture.index = index;
    this->emissiveTexture.texCoord = texCoord;
  }
  void setOcclusionTexture(int index, int texCoord = -1, double strength = 1.0) {
    this->occlusionTexture.index = index;
    this->occlusionTexture.texCoord = texCoord;
    this->occlusionTexture.strength = strength;
  }
  void setEmissiveFactor(double a, double b, double c) {
    this->emissiveFactor[0] = a;
    this->emissiveFactor[1] = b;
    this->emissiveFactor[2] = c;
  }
  std::string getName() { return this->name; }
  struct pbrMetallicRoughness& getPbrMetallicRoughness() { return this->pbrMetallicRoughness; }
  struct normalTexture& getNormalTexture() { return this->normalTexture; }
  struct emissiveTexture& getEmissiveTexture() { return this->emissiveTexture; }
  struct occlusionTexture& getOcclusionTexture() { return this->occlusionTexture; }
  const double* getEmissiveFactor() { return this->emissiveFactor; }

  void fromJson(json& data);
};

class GLTFTexture : public InterpolateGLTexture {
 private:
  GLTFModel* model;
  int sampler;
  int source;
  std::string name;

 public:
  GLTFTexture(GLTFModel* model) : model(model) {}
  void setSampler(int sampler) { this->sampler = sampler; }
  void setSource(int source) { this->source = source; }
  void setName(std::string name) { this->name = name; }
  int getSampler() { return this->sampler; }
  int getSource() { return this->source; }
  std::string getName() { return this->name; }

  void fromJson(json& data);
  void loadImage();
};

class GLTFImage {
 private:
  // TODO add other fileds
  GLTFModel* model;
  std::string uri;

 public:
  GLTFImage(GLTFModel* model) : model(model) {}
  void setUri(std::string& uri) { this->uri = uri; }
  std::string getUri() { return this->uri; }
  void fromJson(json& data);
};

enum class GLTFComponentType {
  BYTE = 5120,            // 8
  UNSIGNED_BYTE = 5121,   // 8
  SHORT = 5122,           // 16
  UNSIGNED_SHORT = 5123,  // 16
  UNSIGNED_INT = 5125,    // 32
  FLOAT = 5126            // 32
};  // namespace qtgl

struct GLTFElementType {
  static const std::string SCALAR;
  static const std::string VEC2;
  static const std::string VEC3;
  static const std::string VEC4;
  static const std::string MAT2;
  static const std::string MAT3;
  static const std::string MAT4;
};

class GLTFAccessor {
 private:
  GLTFModel* model;
  int bufferView;
  int byteOffset;
  GLTFComponentType componentType;
  bool normalized = false;
  int count;
  std::string type;
  std::vector<double> max;
  std::vector<double> min;
  // TODO sparse
  std::string name;
  void* data = nullptr;

 public:
  GLTFAccessor(GLTFModel* model) : model(model) {}
  ~GLTFAccessor() {
    if (data) {
      delete[] data;
      data = nullptr;
    }
  }

  void setBufferView(int bufferView) { this->bufferView = bufferView; }
  void setByteOffset(int byteOffset) { this->byteOffset = byteOffset; }
  void setComponentType(int componentType) {
    this->componentType = static_cast<GLTFComponentType>(componentType);
  }
  void setNormalized(bool normalized) { this->normalized = normalized; }
  void setCount(int count) { this->count = count; }
  void setType(std::string& type) { this->type = type; }
  void setMax(std::vector<double> max) { this->max = max; }
  void setMin(std::vector<double> min) { this->min = min; }
  void setName(std::string& name) { this->name = name; }

  int getBufferView() { return this->bufferView; }
  int getByteOffset() { return this->byteOffset; }
  GLTFComponentType getComponentType() { return this->componentType; }
  bool getNormalized() { return this->normalized; }
  int getCount() { return this->count; }
  std::string getType() { return this->type; }
  std::vector<double>& getMax() { return this->max; }
  std::vector<double>& getMin() { return this->min; }
  std::string getName() { return this->name; }

  void fromJson(json& data);

  void* loadData();

  size_t getElementByteLength() {
    return getComponentSize(this->getComponentType()) * getComponentNumOfElement(this->getType());
  }

  static size_t getComponentSize(GLTFComponentType type) {
    switch (type) {
      case GLTFComponentType::BYTE:
      case GLTFComponentType::UNSIGNED_BYTE:
        return 1;
      case GLTFComponentType::SHORT:
      case GLTFComponentType::UNSIGNED_SHORT:
        return 2;
      case GLTFComponentType::UNSIGNED_INT:
      case GLTFComponentType::FLOAT:
        return 4;
      default:
        return 0;
    }
  }

  static size_t getComponentNumOfElement(std::string type) {
    if (type == GLTFElementType::SCALAR) {
      return 1;
    } else if (type == GLTFElementType::VEC2) {
      return 2;
    } else if (type == GLTFElementType::VEC3) {
      return 3;
    } else if (type == GLTFElementType::VEC4) {
      return 4;
    } else if (type == GLTFElementType::MAT2) {
      return 4;
    } else if (type == GLTFElementType::MAT3) {
      return 9;
    } else if (type == GLTFElementType::MAT4) {
      return 16;
    } else {
      throw std::runtime_error("type error: " + type);
      return 0;
    }
  }
};

class GLTFBufferView {
 private:
  GLTFModel* model;
  int buffer;
  int byteOffset = 0;
  int byteLength;
  int byteStride;
  int target;
  std::string name;
  void* data = nullptr;

 public:
  GLTFBufferView(GLTFModel* model) : model(model) {}
  ~GLTFBufferView() {
    if (data) {
      delete[] data;
      data = nullptr;
    }
  }
  void setBuffer(int buffer) { this->buffer = buffer; }
  void setByteOffset(int byteOffset) { this->byteOffset = byteOffset; }
  void setByteLength(int byteLength) { this->byteLength = byteLength; }
  void setByteStride(int byteStride) { this->byteStride = byteStride; }
  void setTarget(int target) { this->target = target; }
  void setName(std::string name) { this->name = name; }

  int getBuffer() { return this->buffer; }
  int getByteOffset() { return this->byteOffset; }
  int getByteLength() { return this->byteLength; }
  int getByteStride() { return this->byteStride; }
  int getTarget() { return this->target; }
  std::string getName() { return this->name; }

  void fromJson(json& data);

  void* loadData();
};

class GLTFBuffer {
 private:
  GLTFModel* model;
  std::string uri;
  int byteLength;
  std::string name;
  void* data = nullptr;

 public:
  GLTFBuffer(GLTFModel* model) : model(model) {}
  ~GLTFBuffer() {
    if (data) {
      delete[] data;
      data = nullptr;
    }
  }

  void setUri(std::string uri) { this->uri = uri; }
  void setByteLength(int byteLength) { this->byteLength = byteLength; }
  void setName(std::string& name) { this->name = name; }

  std::string getUri() { return this->uri; }
  int getByteLength() { return this->byteLength; }
  std::string getName() { return this->name; }

  void fromJson(json& data);

  void* loadData();
};

class GLTFModel : public GLModel {
 private:
  std::filesystem::path path;
  std::filesystem::path dir;
  GLTFAsset asset;
  int scene;
  std::vector<GLTFScene> scenes;
  std::vector<GLTFNode> nodes;
  std::vector<GLTFMesh> meshes;
  std::vector<GLTFMaterial> materials;
  std::vector<GLTFTexture> textures;
  std::vector<GLTFImage> images;
  std::vector<GLTFAccessor> accessors;
  std::vector<GLTFBufferView> bufferViews;
  std::vector<GLTFBuffer> buffers;

  // temp
  std::vector<GLMaterialBase*> materialBuffer;

  std::vector<GLPrimitive> primitives;

  void parseFromFile(std::string& path);

 public:
  GLTFModel(std::string path) {
    this->path = std::filesystem::path(path);
    this->dir = this->path.parent_path();

    parseFromFile(path);
  }
  ~GLTFModel() {
    for (GLMaterialBase* p : materialBuffer) {
      delete p;
    }
    materialBuffer.clear();
  }
  std::filesystem::path& getPath() { return this->path; }
  std::filesystem::path& getDir() { return this->dir; }
  GLTFAsset& getAsset() { return asset; }
  int getScene() { return this->scene; }
  void addScene(GLTFScene& scene) { scenes.push_back(scene); }
  std::vector<GLTFScene>& getScenes() { return this->scenes; }
  void addNode(GLTFNode& node) { nodes.push_back(node); }
  std::vector<GLTFNode>& getNodes() { return this->nodes; }
  void addMesh(GLTFMesh& mesh) { meshes.push_back(mesh); }
  std::vector<GLTFMesh>& getMeshes() { return this->meshes; }
  void addMaterial(GLTFMaterial& material) { this->materials.push_back(material); }
  std::vector<GLTFMaterial>& getMaterials() { return this->materials; }
  void addTexture(GLTFTexture& texture) { this->textures.push_back(texture); }
  std::vector<GLTFTexture>& getTextures() { return this->textures; }
  void addImage(GLTFImage& image) { this->images.push_back(image); }
  std::vector<GLTFImage>& getImages() { return this->images; }
  void addAccessor(GLTFAccessor& accessor) { this->accessors.push_back(accessor); }
  std::vector<GLTFAccessor>& getAccessors() { return this->accessors; }
  void addBufferView(GLTFBufferView& bufferView) { this->bufferViews.push_back(bufferView); }
  std::vector<GLTFBufferView>& getBufferViews() { return this->bufferViews; }
  void addBuffer(GLTFBuffer& buffer) { this->buffers.push_back(buffer); }
  std::vector<GLTFBuffer>& getBuffers() { return this->buffers; }

  // temp
  std::vector<GLMaterialBase*>& getMaterialBuffer() { return this->materialBuffer; }

  std::vector<GLPrimitive> getPrimitives();
};

}  // namespace qtgl