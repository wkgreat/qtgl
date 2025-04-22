#include "gltfmodel.hpp"
#include "stringutils.hpp"

#include <regex>

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
  this->name = data.contains("name") ? data["name"] : "";

  int index;
  int texCoord;
  json pbrJson = data["pbrMetallicRoughness"];

  if (pbrJson.contains("baseColorTexture")) {
    json pbrBaseColorJson = pbrJson["baseColorTexture"];
    index = pbrBaseColorJson["index"];
    if (pbrBaseColorJson.contains("texCoord")) {
      texCoord = pbrBaseColorJson["texCoord"];
    } else {
      texCoord = 0;
    }
    this->setPbrBaseColorTexture(index, texCoord);
    this->hasPbrBaseColorTexture = true;
  } else {
    this->hasPbrBaseColorTexture = false;
  }

  if (pbrJson.contains("baseColorFactor")) {
    std::vector<double> fs = pbrJson["baseColorFactor"].get<std::vector<double>>();
    this->setPbrBaseColorFactor(fs);
  }

  if (pbrJson.contains("metallicRoughnessTexture")) {
    json pbrMetallicRoughnessJson = pbrJson["metallicRoughnessTexture"];
    index = pbrMetallicRoughnessJson["index"];
    if (pbrMetallicRoughnessJson.contains("texCoord")) {
      texCoord = pbrMetallicRoughnessJson["texCoord"];
    } else {
      texCoord = 0;
    }
    this->setPbrMetallicRoughnessTexture(index, texCoord);
    this->hasPbrMetallicRoughnessTexture = true;
  } else {
    this->hasPbrMetallicRoughnessTexture = false;
  }

  if (pbrJson.contains("metallicFactor")) {
    this->setPbrMetallicFactor(pbrJson["metallicFactor"]);
  }
  if (pbrJson.contains("roughnessFactor")) {
    this->setPbrRoughnessFactor(pbrJson["roughnessFactor"]);
  }

  if (data.contains("normalTexture")) {
    json normalTextureJson = data["normalTexture"];
    index = normalTextureJson["index"];
    double scale = 1.0;
    if (normalTextureJson.contains("texCoord")) {
      texCoord = normalTextureJson["texCoord"];
    } else {
      texCoord = 0;
    }
    if (normalTextureJson.contains("scale")) {
      scale = normalTextureJson["scale"];
    } else {
      scale = 1.0;
    }
    this->setNormalTexture(index, texCoord, scale);
    this->hasNormalTexture = true;
  } else {
    this->hasNormalTexture = false;
  }

  if (data.contains("emissiveTexture")) {
    json emissiveTextureJson = data["emissiveTexture"];
    index = emissiveTextureJson["index"];
    if (emissiveTextureJson.contains("texCoord")) {
      texCoord = emissiveTextureJson["texCoord"];
    } else {
      texCoord = 0;
    }
    this->setEmissiveTexture(index, texCoord);
    this->hasEmissiveTexture = true;
  } else {
    this->hasEmissiveTexture = false;
  }

  if (data.contains("occlusionTexture")) {
    json occlusionTextureJson = data["occlusionTexture"];
    index = occlusionTextureJson["index"];
    if (occlusionTextureJson.contains("texCoord")) {
      texCoord = occlusionTextureJson["texCoord"];
    } else {
      texCoord = 0;
    }
    double strength = 1.0;
    if (occlusionTextureJson.contains("strength")) {
      strength = occlusionTextureJson["strength"];
    } else {
      strength = 1.0;
    }
    this->setOcclusionTexture(index, texCoord, strength);
    this->hasOcclusionTexture = true;
  } else {
    this->hasOcclusionTexture = false;
  }

  if (data.contains("emissiveFactor")) {
    std::vector<double> emissiveFactor = data["emissiveFactor"].get<std::vector<double>>();
    this->setEmissiveFactor(emissiveFactor[0], emissiveFactor[1], emissiveFactor[2]);
  } else {
    this->setEmissiveFactor(0, 0, 0);
  }

  if (data.contains("alphaMode")) {
    this->alphaMode = data["alphaMode"];
  }

  if (data.contains("alphaCutoff")) {
    this->alphaCutoff = data["alphaCutoff"];
  }

  if (data.contains("doubleSided")) {
    this->doubleSided = data["doubleSided"];
  }
}

int GLTFMaterial::getBaseColorTexCoordN() {
  if (this->hasPbrBaseColorTexture) {
    return this->pbrMetallicRoughness.baseColorTexture.texCoord;
  } else {
    return -1;
  }
}
int GLTFMaterial::getMetallocRougnnessTexCoordN() {
  if (this->hasPbrMetallicRoughnessTexture) {
    return this->pbrMetallicRoughness.metallicRoughnessTexture.texCoord;
  } else {
    return -1;
  }
}
int GLTFMaterial::getNormalTexCoordN() {
  if (this->hasNormalTexture) {
    return this->normalTexture.texCoord;
  } else {
    return -1;
  }
}
int GLTFMaterial::getEmissiveTexCoordN() {
  if (this->hasEmissiveTexture) {
    return this->emissiveTexture.texCoord;
  } else {
    return -1;
  }
}
int GLTFMaterial::getOcclusionTexCoordN() {
  if (this->hasOcclusionTexture) {
    return this->occlusionTexture.texCoord;
  } else {
    return -1;
  }
}

Color01 GLTFMaterial::getBaseColor(TexCoord& t) {
  // TODO
  if (!this->hasPbrBaseColorTexture) {
    double* cs = this->pbrMetallicRoughness.baseColorFactor;
    return {cs[0], cs[1], cs[2], 1};
  } else {
    GLTFTexture& texture =
        this->model->getTextures()[this->pbrMetallicRoughness.baseColorTexture.index];
    return texture.sample(t);
  }
}
Eigen::Vector2d GLTFMaterial::getMetallicRoughness(TexCoord& t) {
  if (!this->hasPbrMetallicRoughnessTexture) {
    double m = this->pbrMetallicRoughness.metallicFactor;
    double r = this->pbrMetallicRoughness.roughnessFactor;
    return {m, r};
  } else {
    GLTFTexture& texture =
        this->model->getTextures()[this->pbrMetallicRoughness.metallicRoughnessTexture.index];
    Color01 c = texture.sample(t);  // R: None G: roughness B: metallic
    return {c[2], c[1]};
  }
}
Normal GLTFMaterial::getNormal(TexCoord& t) {
  // TODO
  return {0, 0, 0};
}
Color01 GLTFMaterial::getEmissive(TexCoord& t) {
  if (!this->hasEmissiveTexture) {
    return {0, 0, 0, 1};
  } else {
    GLTFTexture& texture = this->model->getTextures()[this->emissiveTexture.index];
    Color01 c = texture.sample(t);
    return c;
  }
}
double GLTFMaterial::getOcclusion(TexCoord& t) {
  if (!this->hasOcclusionTexture) {
    return 1.0;
  } else {
    GLTFTexture& texture = this->model->getTextures()[this->occlusionTexture.index];
    Color01 c = texture.sample(t);
    return c[0];
  }
}

double GLTFMaterial::getOcclusionStrength() {
  if (!this->hasOcclusionTexture) {
    return 1.0;
  } else {
    return this->getOcclusionTexture().strength;
  }
}

void GLTFTexture::fromJson(json& data) {
  this->name = data.contains("name") ? data["name"] : "";
  this->sampler = data.contains("sampler") ? data["sampler"] : -1;
  this->source = data.contains("source") ? data["source"] : -1;
}

void GLTFTexture::loadImage() {
  std::string uri = this->model->getImages()[this->source].getUri();
  std::regex base64_regex("data:image/[^;]+;base64,(.+)");
  std::smatch match;
  if (std::regex_search(uri, match, base64_regex)) {
    std::string base64_data = match[1];
    std::vector<uchar> image_data = stringutils::base64_decode(base64_data);
    this->mat = cv::imdecode(image_data, cv::IMREAD_COLOR);
  } else {
    std::filesystem::path puri(uri);
    std::filesystem::path fullpath = this->model->getDir() / puri;
    this->mat = cv::imread(fullpath.string());
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

  // texture
  for (json textureJson : data["textures"]) {
    GLTFTexture texture(this);
    texture.fromJson(textureJson);
    this->addTexture(texture);
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

  for (GLTFTexture& texture : this->getTextures()) {
    texture.loadImage();
  }
}

template <typename T>
static void loadIndicesFromAccessor(GLTFAccessor& accessor, Indices3& indices) {
  int ntriangles = accessor.getCount() / 3;
  indices.conservativeResize(ntriangles, Eigen::NoChange);
  T* idxdata = reinterpret_cast<T*>(accessor.loadData());
  for (int i = 0; i < ntriangles; ++i) {
    Index3 idx(idxdata[i * 3], idxdata[i * 3 + 1], idxdata[i * 3 + 2]);
    indices.row(i) = idx;
  }
}

static GLPrimitive fromGLTFPrimitve(GLTFModel* model, GLTFPrimitive* p, Eigen::Matrix4d& mtx) {
  int mode = p->getMode();

  if (mode == 4) {
    if (p->getIndices() != -1) {
      // 这里indices的数据类型可能是两种 unsigned short || unsigned int
      GLTFAccessor& indicesAccessor = model->getAccessors()[p->getIndices()];
      Indices3 indices;
      if (indicesAccessor.getCount() % 3 != 0) {
        spdlog::error("绘制三角形，但是索引数量不是3的倍数");
      }
      if (indicesAccessor.getComponentType() == GLTFComponentType::UNSIGNED_SHORT) {
        loadIndicesFromAccessor<uint16_t>(indicesAccessor, indices);
      } else if (indicesAccessor.getComponentType() == GLTFComponentType::UNSIGNED_INT) {
        loadIndicesFromAccessor<uint32_t>(indicesAccessor, indices);
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

      // material
      GLMaterialBase* material = &(model->getMaterials()[p->getMaterial()]);

      MeshType meshType = static_cast<MeshType>(mode);
      GLPrimitive prim(meshType, indices, worldPos, worldNorm, material);

      // texcoords (float/ubyte normalized/ushort normalized)
      for (auto pair : p->getAttributes()) {
        if (stringutils::start_with(pair.first, "TEXCOORD")) {
          std::vector<std::string> ss = stringutils::split(pair.first, '_');
          int k = stringutils::string2int(ss.back());
          int v = pair.second;
          TexCoords texcoords;

          GLTFAccessor& texCoordAccessor = model->getAccessors()[v];
          if (texCoordAccessor.getType() != GLTFElementType::VEC2) {
            spdlog::error("texcoord的accessor元素类型错误! {}", texCoordAccessor.getType());
          }
          if (texCoordAccessor.getComponentType() != GLTFComponentType::BYTE &&
              texCoordAccessor.getComponentType() != GLTFComponentType::UNSIGNED_SHORT &&
              texCoordAccessor.getComponentType() != GLTFComponentType::FLOAT) {
            spdlog::error("texcoord的accessor数据类型错误! {}",
                          static_cast<int>(texCoordAccessor.getComponentType()));
          }
          int ntexcoord = texCoordAccessor.getCount();
          texcoords.conservativeResize(ntexcoord, Eigen::NoChange);
          if (texCoordAccessor.getComponentType() == GLTFComponentType::BYTE) {
            uint8_t* data = reinterpret_cast<uint8_t*>(texCoordAccessor.loadData());
            for (int i = 0; i < ntexcoord; ++i) {
              TexCoord texcoord(data[i * 2] / 255.0, data[i * 2 + 1] / 255.0);
              texcoords.row(i) = texcoord;
            }
          }
          if (texCoordAccessor.getComponentType() == GLTFComponentType::UNSIGNED_INT) {
            uint32_t* data = reinterpret_cast<uint32_t*>(texCoordAccessor.loadData());
            for (int i = 0; i < ntexcoord; ++i) {
              TexCoord texcoord(data[i * 2] / 4294967295.0, data[i * 2 + 1] / 4294967295.0);
              texcoords.row(i) = texcoord;
            }
          }
          if (texCoordAccessor.getComponentType() == GLTFComponentType::FLOAT) {
            float* data = reinterpret_cast<float*>(texCoordAccessor.loadData());
            for (int i = 0; i < ntexcoord; ++i) {
              // GLTF默认坐标原点为左上角，但是实际渲染时使用原点为左下角的坐标系
              // 所以UV中的V需要反转
              TexCoord texcoord(data[i * 2], 1 - data[i * 2 + 1]);
              texcoords.row(i) = texcoord;
            }
          }

          prim.addTexCoord(k, texcoords);
        }
      }

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