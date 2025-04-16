#pragma once

#include <fstream>
#include <iostream>
#include <map>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

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
  std::string getName() { return this->name; }
  union transform& getTransform() { return transform; }
  bool getHasTransform() const { return hasTransform; }
  bool getIsTransformSep() const { return isTransformSeperate; }
  std::vector<int>& getChildren() { return children; }

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
  int getMaterial() const { return this->material; }
  int getIndices() const { return this->indices; }
  int getAttribute(std::string& k) { return this->attributes[k]; }
};

class GLTFMaterial {
 private:
  GLTFModel* model;
  std::string name;
  struct pbrMetallicRoughness {
    // TODO baseColorFactor metallicFactor roughnessFactor
    struct {
      int index;
      int texCoord = -1;
    } baseColorTexture;
    struct {
      int index;
      int texCoord = -1;
    } metallicRoughnessTexture;
  } pbrMetallicRoughness;
  struct normalTexture {
    double scale = 1.0;
    int index;
    int texCoord = -1;
  } normalTexture;
  struct emissiveTexture {
    int index;
    int texCoord = -1;
  } emissiveTexture;
  struct occlusionTexture {
    double strength = 1.0;
    int index;
    int texCoord = -1;
  } occlusionTexture;
  double emissiveFactor[3];

 public:
  GLTFMaterial(GLTFModel* model) : model(model) {}
  void setName(std::string& name) { this->name = name; }
  void setPbrBaseColorTexture(int index, int texCoord = -1) {
    this->pbrMetallicRoughness.baseColorTexture.index = index;
    this->pbrMetallicRoughness.baseColorTexture.texCoord = texCoord;
  }
  void setPbrMetallicRoughnessTexture(int index, int texCoord = -1) {
    this->pbrMetallicRoughness.metallicRoughnessTexture.index = index;
    this->pbrMetallicRoughness.metallicRoughnessTexture.texCoord = texCoord;
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

class GLTFImage {
 private:
  // TODO add other fileds
  GLTFModel* model;
  std::string uri;

 public:
  GLTFImage(GLTFModel* model) : model(model) {}
  void setUri(std::string& uri) { this->uri = uri; }
  void fromJson(json& data);
};

class GLTFAccessor {
 private:
  GLTFModel* model;
  int bufferView;
  int byteOffset;
  int componentType;
  bool normalized = false;
  int count;
  std::string type;
  std::vector<double> max;
  std::vector<double> min;
  // TODO sparse
  std::string name;

 public:
  GLTFAccessor(GLTFModel* model) : model(model) {}

  void setBufferView(int bufferView) { this->bufferView = bufferView; }
  void setByteOffset(int byteOffset) { this->byteOffset = byteOffset; }
  void setComponentType(int componentType) { this->componentType = componentType; }
  void setNormalized(bool normalized) { this->normalized = normalized; }
  void setCount(int count) { this->count = count; }
  void setType(std::string& type) { this->type = type; }
  void setMax(std::vector<double> max) { this->max = max; }
  void setMin(std::vector<double> min) { this->min = min; }
  void setName(std::string& name) { this->name = name; }

  int getBufferView() { return this->bufferView; }
  int getByteOffset() { return this->byteOffset; }
  int getComponentType() { return this->componentType; }
  bool getNormalized() { return this->normalized; }
  int getCount() { return this->count; }
  std::string getType() { return this->type; }
  std::vector<double>& getMax() { return this->max; }
  std::vector<double>& getMin() { return this->min; }
  std::string getName() { return this->name; }

  void fromJson(json& data);
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

 public:
  GLTFBufferView(GLTFModel* model) : model(model) {}
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
};

class GLTFBuffer {
 private:
  GLTFModel* model;
  std::string uri;
  int byteLength;
  std::string name;

 public:
  GLTFBuffer(GLTFModel* model) : model(model) {}

  void setUri(std::string uri) { this->uri = uri; }
  void setByteLength(int byteLength) { this->byteLength = byteLength; }
  void setName(std::string& name) { this->name = name; }

  std::string getUri() { return this->uri; }
  int getByteLength() { return this->byteLength; }
  std::string getName() { return this->name; }

  void fromJson(json& data);
};

class GLTFModel {
 private:
  std::string path;
  GLTFAsset asset;
  int scene;
  std::vector<GLTFScene> scenes;
  std::vector<GLTFNode> nodes;
  std::vector<GLTFMesh> meshes;
  std::vector<GLTFMaterial> materials;
  std::vector<GLTFImage> images;
  std::vector<GLTFAccessor> accessors;
  std::vector<GLTFBufferView> bufferViews;
  std::vector<GLTFBuffer> buffers;

  void parseFromFile(std::string& path);

 public:
  GLTFModel(std::string path) : path(path) { parseFromFile(path); }
  ~GLTFModel() = default;
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
  void addImage(GLTFImage& image) { this->images.push_back(image); }
  std::vector<GLTFImage>& getImages() { return this->images; }
  void addAccessor(GLTFAccessor& accessor) { this->accessors.push_back(accessor); }
  std::vector<GLTFAccessor>& getAccessors() { return this->accessors; }
  void addBufferView(GLTFBufferView& bufferView) { this->bufferViews.push_back(bufferView); }
  std::vector<GLTFBufferView>& getBufferViews() { return this->bufferViews; }
  void addBuffer(GLTFBuffer& buffer) { this->buffers.push_back(buffer); }
  std::vector<GLTFBuffer> getBuffers() { return this->buffers; }
};

};  // namespace qtgl