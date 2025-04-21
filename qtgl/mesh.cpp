#include "mesh.hpp"

namespace qtgl {

GLMeshGroup::GLMeshGroup(GLMesh* parent, std::string& name) {
  this->parent = parent;
  this->name = name;
}
GLMeshGroup::~GLMeshGroup() = default;

GLMesh* GLMeshGroup::getParent() { return parent; }
void GLMeshGroup::setParent(GLMesh* parent) {
  assert(editing);
  this->parent = parent;
}
Indices3& GLMeshGroup::getIndices() { return indices; }
void GLMeshGroup::setIndices(Indices3& indices) {
  assert(editing);
  this->indices = indices;
}
NormIndices& GLMeshGroup::getNormIndices() { return normIndices; }
void GLMeshGroup::setNormIndices(NormIndices& normIndices) {
  assert(editing);
  this->normIndices = normIndices;
}
std::vector<std::vector<Color01>>& GLMeshGroup::getColors() { return colors; }
void GLMeshGroup::setColors(std::vector<std::vector<Color01>>& colors) {
  assert(editing);
  this->colors = colors;
}
std::vector<TexRef>& GLMeshGroup::getTexRefs() { return texrefs; }
void GLMeshGroup::setTexRefs(std::vector<TexRef>& texrefs) {
  assert(editing);
  this->texrefs = texrefs;
}

void GLMeshGroup::addIndex3(Index3 idx, Color01 clr0, Color01 clr1, Color01 clr2) {
  assert(editing);
  indices.conservativeResize(indices.rows() + 1, indices.cols());
  indices.row(indices.rows() - 1) = idx;
  std::vector<Color01> tricolor{clr0, clr1, clr2};
  colors.push_back(std::move(tricolor));
}

void GLMeshGroup::addNormIndex(NormIndex idx) {
  assert(editing);
  normIndices.conservativeResize(normIndices.rows() + 1, normIndices.cols());
  normIndices.row(normIndices.rows() - 1) = idx;
}

GLObject* GLMeshGroup::clone() {
  GLMeshGroup* g = new GLMeshGroup(this->parent, this->name);
  g->indices = indices;
  g->normIndices = normIndices;
  g->colors = colors;
  g->texrefs = texrefs;
  g->editing = this->editing;
  return g;
}
void GLMeshGroup::draw(QPainter& painter) {
  // TODO
}

void GLMeshGroup::transformVerticesToScreen(Eigen::Matrix4d& mtx) {
  // TODO
}
void GLMeshGroup::transformWithModelMatrix() {
  // TODO
}

void GLMeshGroup::drawSkeleton(QPainter& painter) {
  int n = indices.rows();
  for (int i = 0; i < n; ++i) {
    Index3 idx = indices.row(i);
    Vertice p1 = vertices.row(idx[0]);
    Vertice p2 = vertices.row(idx[1]);
    painter.drawLine(p1[0], p1[1], p2[0], p2[1]);

    p1 = vertices.row(idx[1]);
    p2 = vertices.row(idx[2]);
    painter.drawLine(p1[0], p1[1], p2[0], p2[1]);

    p1 = vertices.row(idx[2]);
    p2 = vertices.row(idx[0]);
    painter.drawLine(p1[0], p1[1], p2[0], p2[1]);
  }
}

void GLMeshGroup::startEditing() { editing = true; }
void GLMeshGroup::finishEditing() { editing = false; }

void GLMeshGroup::addIndex3(Index3 idx) {
  assert(editing);
  addIndex3(idx, GLMesh::defaultColor, GLMesh::defaultColor, GLMesh::defaultColor);
}

// void GLMeshGroup::rasterize(GLScene& scene) {
//   int n = indices.rows();
//   for (int i = 0; i < n; ++i) {
//     Index3 idx = indices.row(i);
//     NormIndex normIdx = normIndices.row(i);
//     TexRef ref = texrefs[i];
//     Vertice rp0 = parent->getVertices().row(idx[0]);
//     Vertice rp1 = parent->getVertices().row(idx[1]);
//     Vertice rp2 = parent->getVertices().row(idx[2]);
//     Vertice p0 = parent->getScreenVertices().row(idx[0]);
//     Vertice p1 = parent->getScreenVertices().row(idx[1]);
//     Vertice p2 = parent->getScreenVertices().row(idx[2]);
//     Normal n0 = parent->getTransformedNormals().row(normIdx[0]).normalized();
//     Normal n1 = parent->getTransformedNormals().row(normIdx[1]).normalized();
//     Normal n2 = parent->getTransformedNormals().row(normIdx[2]).normalized();

//     GLMaterial* material = parent->getMaterial(ref.mtlname);

//     Triangle3 t(p0, p1, p2, n0, n1, n2);

//     if (ref.indices[0] != -1 && ref.indices[1] != -1 && ref.indices[2] != -1) {
//       TexCoord t0 = parent->getTexCoords().row(ref.indices[0]);
//       TexCoord t1 = parent->getTexCoords().row(ref.indices[1]);
//       TexCoord t2 = parent->getTexCoords().row(ref.indices[2]);
//       t.addTexCoords(t0, t1, t2);
//     }
//     t.setWorldPos(rp0, rp1, rp2);
//     rasterizeTriangle(scene, t, material);
//   }
// }

// void GLMeshGroup::rasterizeTriangle(GLScene& scene, Triangle3& t, GLMaterial* material) {
//   GLTriangleShader shader;
//   shader.shade(t, material, scene.getCamera(), scene.getLights(), scene.getShadows(),
//                scene.getAmbient(), scene.getFragments(), scene.getInvTranformMatrix());
// }

std::vector<GLPrimitive> GLMeshGroup::getPrimitives() {
  int n = indices.rows();
  Indices3 idxes;
  Vertices worldPos;
  Normals normals;
  GLMaterial* pm = nullptr;
  GLMaterial* cm = nullptr;
  TexCoords texcoords;
  bool hasTexcoords = false;
  std::vector<GLPrimitive> ps;
  int in = 0;
  int cn = 0;

  idxes.conservativeResize(n, Eigen::NoChange);
  worldPos.conservativeResize(n * 3, Eigen::NoChange);
  normals.conservativeResize(n * 3, Eigen::NoChange);
  texcoords.conservativeResize(n * 3, Eigen::NoChange);

  for (int i = 0; i < n; ++i) {
    TexRef ref = texrefs[i];
    Index3 idx = indices.row(i);
    NormIndex normIdx = normIndices.row(i);
    cm = parent->getMaterial(ref.mtlname);

    if (cm != pm) {
      if (in != 0) {
        idxes.conservativeResize(in, Eigen::NoChange);
        worldPos.conservativeResize(cn, Eigen::NoChange);
        normals.conservativeResize(cn, Eigen::NoChange);
        GLPrimitive p(MeshType::TRIANGLES, idxes, worldPos, normals, pm);
        if (hasTexcoords) {
          texcoords.conservativeResize(cn, Eigen::NoChange);
          p.addTexCoord(0, texcoords);
        }
        ps.push_back(p);
        in = 0;
        cn = 0;
        idxes.conservativeResize(n, Eigen::NoChange);
        worldPos.conservativeResize(3 * n, Eigen::NoChange);
        texcoords.conservativeResize(3 * n, Eigen::NoChange);
        normals.conservativeResize(3 * n, Eigen::NoChange);
      }
    }

    Vertice rp0 = parent->getVertices().row(idx[0]);
    Vertice rp1 = parent->getVertices().row(idx[1]);
    Vertice rp2 = parent->getVertices().row(idx[2]);
    Vertice p0 = parent->getScreenVertices().row(idx[0]);
    Vertice p1 = parent->getScreenVertices().row(idx[1]);
    Vertice p2 = parent->getScreenVertices().row(idx[2]);
    Normal n0 = parent->getTransformedNormals().row(normIdx[0]).normalized();
    Normal n1 = parent->getTransformedNormals().row(normIdx[1]).normalized();
    Normal n2 = parent->getTransformedNormals().row(normIdx[2]).normalized();

    idxes(in, 0) = cn;
    idxes(in, 1) = cn + 1;
    idxes(in, 2) = cn + 2;
    worldPos.row(cn) = rp0;
    worldPos.row(cn + 1) = rp1;
    worldPos.row(cn + 2) = rp2;
    normals.row(cn + 0) = n0;
    normals.row(cn + 1) = n1;
    normals.row(cn + 2) = n2;

    if (ref.indices[0] != -1 && ref.indices[1] != -1 && ref.indices[2] != -1) {
      hasTexcoords = true;
      TexCoord t0 = parent->getTexCoords().row(ref.indices[0]);
      TexCoord t1 = parent->getTexCoords().row(ref.indices[1]);
      TexCoord t2 = parent->getTexCoords().row(ref.indices[2]);
      texcoords.row(cn + 0) = t0;
      texcoords.row(cn + 1) = t1;
      texcoords.row(cn + 2) = t2;
    }
    pm = cm;
    in += 1;
    cn += 3;
  }

  if (in != 0) {
    idxes.conservativeResize(in, Eigen::NoChange);
    worldPos.conservativeResize(cn, Eigen::NoChange);
    normals.conservativeResize(cn, Eigen::NoChange);
    GLPrimitive p(MeshType::TRIANGLES, idxes, worldPos, normals, cm);
    if (hasTexcoords) {
      texcoords.conservativeResize(n * 3, Eigen::NoChange);
      p.addTexCoord(0, texcoords);
    }
    ps.push_back(p);
    in = 0;
    cn = 0;
  }

  return ps;
}

const Color01 GLMesh::defaultColor = {1, 1, 1, 1};
const std::string GLMesh::defaultGroup = "default";

// void GLMesh::rasterize(GLScene& scene) {
//   for (auto g : groups) {
//     (g.second)->rasterize(scene);
//   }
// }

std::vector<GLPrimitive> GLMesh::getPrimitives() {
  std::vector<GLPrimitive> primitvies;
  for (auto g : groups) {
    std::vector<GLPrimitive> ps = (g.second)->getPrimitives();
    primitvies.insert(primitvies.end(), ps.begin(), ps.end());
  }
  return primitvies;
}

GLMesh* GLMesh::fromObjModel(ObjModel* model) {
  GLMesh* mesh = new GLMesh;
  mesh->startEditing();
  mesh->vertices = model->vertices;
  mesh->normals = model->normals;
  mesh->texcoords = model->texcoords;
  for (auto g : model->groups) {
    std::string name = g.first;
    ObjModelGroup& group = g.second;
    GLMeshGroup* meshGroup = new GLMeshGroup(mesh, name);
    meshGroup->startEditing();
    meshGroup->setIndices(group.indices);
    meshGroup->setNormIndices(group.normIndices);
    meshGroup->setTexRefs(group.texrefs);
    mesh->groups[name] = meshGroup;
    Color01 white = {1, 1, 1, 1};
    std::vector<std::vector<Color01>> colors(meshGroup->getIndices().rows(), {white, white, white});
    meshGroup->setColors(colors);
    meshGroup->finishEditing();
  }
  if (model->mtllib) {
    for (auto mtl : model->mtllib->mtls) {
      std::string name = mtl.first;
      ObjMaterial& material = mtl.second;
      mesh->materials[name] = mtl.second.toGLMaterial();
    }
  }
  mesh->finishEditing();
  return mesh;
}

GLMesh* GLMesh::readFromObjFile(std::string fpath) {
  GLMesh* mesh = nullptr;
  ObjModel* model = ObjModel::loadObj(fpath);
  if (model) {
    mesh = fromObjModel(model);
    delete model;
  }
  return mesh;
}

GLMesh::~GLMesh() {
  for (auto g : groups) {
    delete g.second;
  }
  for (auto m : materials) {
    delete m.second;
  }
};
GLMesh::GLMesh(const GLMesh& mesh) : GLObject(mesh) {
  groups = mesh.groups;
  normals = mesh.normals;
  transfromedNormals = mesh.transfromedNormals;
  texcoords = mesh.texcoords;
  // textures = mesh.textures;
  materials = mesh.materials;  // TODO: deepcopy?
}
GLObject* GLMesh::clone() {
  GLMesh* p = new GLMesh;
  p->groups = this->groups;
  p->vertices = this->vertices;
  p->normals = this->normals;
  for (auto g : groups) {
    p->groups[g.first] = reinterpret_cast<GLMeshGroup*>((g.second)->clone());
    p->groups[g.first]->setParent(p);
  }
  p->texcoords = this->texcoords;
  p->materials = this->materials;  // TODO deepcopy?
  p->modelMatrix = this->modelMatrix;
  p->transfromedVertices = this->transfromedVertices;
  p->transfromedNormals = this->transfromedNormals;
  p->screenVertices = this->screenVertices;
  p->editing = this->editing;
  return p;
}

GLMeshGroup* GLMesh::getGroup(std::string& name) {
  if (groups.count(name)) {
    return groups[name];
  } else {
    groups[name] = new GLMeshGroup(this, name);
    return groups[name];
  }
}
std::map<std::string, GLMeshGroup*>& GLMesh::getGroups() { return this->groups; }

Normals& GLMesh::getNormals() { return normals; }
Normals& GLMesh::getTransformedNormals() { return transfromedNormals; }
TexCoords& GLMesh::getTexCoords() { return texcoords; }
GLMaterial* GLMesh::getMaterial(std::string name) {
  if (materials.count(name)) {
    return materials[name];
  } else {
    return nullptr;
  }
}

std::map<std::string, GLMaterial*>& GLMesh::getMaterials() { return materials; }

void GLMesh::addIndex3(Index3 idx) {
  assert(editing);
  addIndex3(const_cast<std::string&>(defaultGroup), idx);
}
void GLMesh::addIndex3(Index3 idx, Color01 clr0, Color01 clr1, Color01 clr2) {
  assert(editing);
  addIndex3(const_cast<std::string&>(defaultGroup), idx, clr0, clr1, clr2);
}
void GLMesh::addIndex3(std::string& groupName, Index3 idx) {
  assert(editing);
  getGroup(groupName)->addIndex3(idx, GLMesh::defaultColor, GLMesh::defaultColor,
                                 GLMesh::defaultColor);
}

void GLMesh::addIndex3(std::string& groupName, Index3 idx, Color01 clr0, Color01 clr1,
                       Color01 clr2) {
  assert(editing);
  GLMeshGroup* group = getGroup(groupName);
  Indices3& indices = group->getIndices();
  indices.conservativeResize(indices.rows() + 1, indices.cols());
  indices.row(indices.rows() - 1) = idx;
  std::vector<Color01> tricolor{clr0, clr1, clr2};
  group->getColors().push_back(std::move(tricolor));
}

void GLMesh::pushNormal(double a, double b, double c) {
  assert(editing);
  Normal n(a, b, c);
  normals.conservativeResize(normals.rows() + 1, normals.cols());
  normals.row(normals.rows() - 1) = n;
}

void GLMesh::addNormIndex(std::string& groupName, NormIndex idx) {
  assert(editing);
  GLMeshGroup* group = getGroup(groupName);
  group->addNormIndex(idx);
}

void GLMesh::pushTexCoord(double u, double v) {
  assert(editing);
  TexCoord t(u, v);
  texcoords.conservativeResize(texcoords.rows() + 1, texcoords.cols());
  texcoords.row(texcoords.rows() - 1) = t;
}

void GLMesh::transformVerticesToScreen(Eigen::Matrix4d& mtx) {
  assert(!editing);
  this->screenVertices = AffineUtils::affine(this->transfromedVertices, mtx);
}

void GLMesh::transformWithModelMatrix() {
  this->transfromedVertices = AffineUtils::affine(this->vertices, this->modelMatrix);
  Eigen::Matrix3d m = this->modelMatrix.block(0, 0, 3, 3);
  this->transfromedNormals = AffineUtils::norm_affine(this->normals, m);
}

void GLMesh::draw(QPainter& painter) {
  // rasterize(painter);
  // drawSkeleton(painter);
}

void GLMesh::startEditing() {
  for (auto g : this->groups) {
    g.second->startEditing();
  }
  editing = true;
}

void GLMesh::finishEditing() {
  this->transformWithModelMatrix();
  for (auto g : this->groups) {
    g.second->finishEditing();
  }
  editing = false;
}

}  // namespace qtgl