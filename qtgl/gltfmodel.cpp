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

  if (data.contains("normalTexture")) {
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
  }

  if (data.contains("emissiveTexture")) {
    json emissiveTextureJson = data["emissiveTexture"];
    index = emissiveTextureJson["index"];
    texCoord = -1;
    if (emissiveTextureJson.contains("texCoord")) {
      texCoord = emissiveTextureJson["texCoord"];
    }
    this->setEmissiveTexture(index, texCoord);
  }

  if (data.contains("occlusionTexture")) {
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
  }

  if (data.contains("emissiveFactor")) {
    std::vector<double> emissiveFactor = data["emissiveFactor"].get<std::vector<double>>();
    this->setEmissiveFactor(emissiveFactor[0], emissiveFactor[1], emissiveFactor[2]);
  }
}

void GLTFImage::fromJson(json& data) {
  // TODO add other fields
  std::string uri = data["uri"];
  this->setUri(uri);
  // TODO read image
}

const std::string GLTFElementType::SCALAR = "SCALAR";
const std::string GLTFElementType::VEC2 = "VEC2";
const std::string GLTFElementType::VEC3 = "VEC3";
const std::string GLTFElementType::VEC4 = "VEC4";
const std::string GLTFElementType::MAT2 = "MAT2";
const std::string GLTFElementType::MAT3 = "MAT3";
const std::string GLTFElementType::MAT4 = "MAT4";

void GLTFAccessor::fromJson(json& data) {
  int bufferView = data.contains("bufferView") ? data["bufferView"] : 0;
  this->setBufferView(bufferView);
  int bufferOffset = data.contains("byteOffset") ? data["byteOffset"] : 0;
  this->setByteOffset(bufferOffset);
  int componentType = data["componentType"];
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

void* GLTFAccessor::loadData() {
  if (!this->data) {
    GLTFBufferView& thebufferview = this->model->getBufferViews()[this->bufferView];
    GLTFBuffer& thebuffer = this->model->getBuffers()[thebufferview.getBuffer()];
    char* rawdata = reinterpret_cast<char*>(thebuffer.loadData());
    if (!rawdata) {
      throw std::runtime_error("获取数据失败");
    }
    int totoffset = this->getByteOffset() + thebufferview.getByteOffset();
    rawdata += totoffset;
    int elemlen = this->getElementByteLength();
    int stride = thebufferview.getByteStride();
    if (stride == -1) {
      stride = elemlen;
    }
    int count = this->getCount();
    int totByteLength = elemlen * count;

    spdlog::trace(
        "GLTFAccessor totoffset: {}, elemlen: {}, stride: {}, count: {}, totByteLength: {}",
        totoffset, elemlen, stride, count, totByteLength);

    char* newdata = new char[totByteLength];

    for (int i = 0, p = 0, q = 0; i < count; i += 1, p += stride, q += elemlen) {
      memcpy(newdata + q, rawdata + p, elemlen);
    }

    this->data = newdata;
  }
  return this->data;
};

void GLTFBufferView::fromJson(json& data) {
  int buffer = data["buffer"];
  this->setBuffer(buffer);
  int byteOffset = data.contains("byteOffset") ? data["byteOffset"] : 0;
  this->setByteOffset(byteOffset);
  int byteLength = data["byteLength"];
  this->setByteLength(byteLength);
  int byteStride = data.contains("byteStride") ? data["byteStride"] : -1;
  this->setByteStride(byteStride);
  int target = data.contains("target") ? data["target"] : -1;
  this->setTarget(target);
  std::string name = data.contains("name") ? data["name"] : "";
  this->setName(name);
}

void* GLTFBufferView::loadData() { return nullptr; }

void GLTFBuffer::fromJson(json& data) {
  std::string uri = data.contains("uri") ? data["uri"] : "";
  this->setUri(uri);
  int byteLength = data["byteLength"];
  this->setByteLength(byteLength);
  std::string name = data.contains("name") ? data["name"] : "";
  this->setName(name);
}

void* GLTFBuffer::loadData() {
  if (!this->data) {
    char* newdata = new char[this->byteLength];
    // TODO 判断URI是文件还是data uri
    std::filesystem::path filepath(this->uri);
    std::filesystem::path fullpath = this->model->getDir() / filepath;
    spdlog::trace("buffer fullpath: {}", fullpath.string());
    std::ifstream file(fullpath, std::ios::binary);
    if (!file.read(reinterpret_cast<char*>(newdata), this->byteLength)) {
      throw std::runtime_error("读取文件失败: " + this->uri);
      // TODO 尝试读取 data uri
    }
    file.close();
    this->data = newdata;
  }
  return this->data;
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

  // load data
  for (GLTFBuffer& buffer : this->getBuffers()) {
    buffer.loadData();
  }
  for (GLTFBufferView& bufferView : this->getBufferViews()) {
    bufferView.loadData();
  }
  for (GLTFAccessor& accessor : this->getAccessors()) {
    accessor.loadData();
  }
}

static GLPrimitive fromGLTFPrimitve(GLTFModel* model, GLTFPrimitive* p, Eigen::Matrix4d& mtx) {
  int mode = p->getMode();

  if (mode == 4) {
    if (p->getIndices() != -1) {
      GLTFAccessor& indicesAccessor = model->getAccessors()[p->getIndices()];
      if (indicesAccessor.getCount() % 3 != 0) {
        spdlog::error("绘制三角形，但是索引数量不是3的倍数");
      }
      uint16_t* idxdata = reinterpret_cast<uint16_t*>(indicesAccessor.loadData());
      int ntriangles = indicesAccessor.getCount() / 3;
      Indices3 indices;
      indices.conservativeResize(ntriangles, Eigen::NoChange);
      for (int i = 0; i < ntriangles; ++i) {
        Index3 idx(idxdata[i * 3], idxdata[i * 3 + 1], idxdata[i * 3 + 2]);
        indices.row(i) = idx;
      }

      // position (float vec3)
      GLTFAccessor& posAccessor = model->getAccessors()[p->getAttribute("POSITION")];
      int npos = posAccessor.getCount();
      if (indicesAccessor.getCount() % 3 != 0) {
        spdlog::error("坐标点维度不为3");
      }
      float* posdata = reinterpret_cast<float*>(posAccessor.loadData());
      Vertices localpos;
      localpos.conservativeResize(npos, Eigen::NoChange);
      for (int i = 0; i < npos; ++i) {
        Vertice pos(static_cast<double>(posdata[i * 3]), static_cast<double>(posdata[i * 3 + 1]),
                    static_cast<double>(posdata[i * 3 + 2]), 1.0);
        localpos.row(i) = pos;
      }
      Vertices worldPos = AffineUtils::affine(localpos, mtx);

      // normal (float vec3)
      GLTFAccessor& normAccessor = model->getAccessors()[p->getAttribute("NORMAL")];
      float* normdata = reinterpret_cast<float*>(normAccessor.loadData());
      int nnorm = normAccessor.getCount();
      if (normAccessor.getCount() % 3 != 0) {
        spdlog::error("法向量维度不为3");
      }
      Normals localnorm;
      localnorm.conservativeResize(nnorm, Eigen::NoChange);
      for (int i = 0; i < nnorm; ++i) {
        Normal norm(static_cast<double>(normdata[i * 3]), static_cast<double>(normdata[i * 3 + 1]),
                    static_cast<double>(normdata[i * 3 + 2]));
        localnorm.row(i) = norm;
      }
      Normals worldNorm = AffineUtils::norm_affine(localnorm, mtx.block(0, 0, 3, 3));

      // texcoords (float/ubyte normalized/ushort normalized)
      // others

      /*
        GLPrimitive(MeshType type, Indices3& indices, Vertices& worldPos, Normals& normal,
              GLMaterial* material)
      */
      MeshType meshType = static_cast<MeshType>(mode);
      std::vector<GLMaterialBase*>& materialBuffer = model->getMaterialBuffer();
      if (materialBuffer.empty()) {
        materialBuffer.push_back(new GLMaterialConstant({0.4, 0.5, 0.6, 1}));
      }
      GLPrimitive prim(meshType, indices, worldPos, worldNorm, materialBuffer.back());
      return prim;

    } else {
      // TODO when indices not defined
      spdlog::warn("indices 未定义.");
      throw std::runtime_error("indices 未定义.");
    }
  } else {
    // TODO other mode.
    spdlog::warn("暂时不支持其他mode.");
    throw std::runtime_error("暂时不支持其他mode.");
  }
}

static void getPrimitivesFromMesh(GLTFModel* model, GLTFMesh& mesh, Eigen::Matrix4d& mtx,
                                  std::vector<GLPrimitive>& primitives) {
  for (GLTFPrimitive* p : mesh.getPrimitives()) {
    primitives.push_back(fromGLTFPrimitve(model, p, mtx));
  }
};

static void getPrimitivesFromNode(GLTFModel* model, GLTFNode& node, Eigen::Matrix4d& mtx,
                                  std::vector<GLPrimitive>& primitives) {
  Eigen::Matrix4d localMtx;
  if (node.getIsTransformSep()) {
    // TODO 三数组组合成转换矩阵
  } else {
    Eigen::Matrix4d m(node.getTransform().matrix);
    localMtx = m.transpose();  // GTLF列主序改成行主序
  }
  mtx = mtx * localMtx;  // 添加转换矩阵
  int imesh = node.getMesh();
  GLTFMesh& mesh = model->getMeshes()[imesh];
  getPrimitivesFromMesh(model, mesh, mtx, primitives);
  for (int icnode : node.getChildren()) {
    GLTFNode& cnode = node.getModel()->getNodes()[icnode];
    getPrimitivesFromNode(model, cnode, mtx, primitives);
  }

  mtx = mtx * localMtx.inverse();  // 撤回转换矩阵
};

std::vector<GLPrimitive> GLTFModel::getPrimitives() {
  if (this->primitives.empty()) {
    // root
    GLTFScene& root = this->scenes[this->scene];
    Eigen::Matrix4d mtx = Eigen::Matrix4d::Identity();
    for (int inode : root.getNodes()) {
      GLTFNode& node = this->nodes[inode];
      getPrimitivesFromNode(this, node, mtx, this->primitives);
    }
  }
  return this->primitives;
}

};  // namespace qtgl