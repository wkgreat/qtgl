#pragma once

#include <QPainter>
#include <QString>
#include <QStringList>
#include <fstream>
#include <iostream>
#include <map>
#include "affineutils.hpp"
#include "material.hpp"
#include "objmodel.hpp"
#include "scene.hpp"
#include "shader.hpp"
#include "texture.hpp"

namespace qtgl {

class GLScene;

class GLObject {
 protected:
  Vertices vertices;
  Eigen::Matrix4d modelMatrix = Eigen::Matrix4d::Identity();
  Vertices transfromedVertices;

 public:
  GLObject() = default;
  virtual ~GLObject() = default;
  GLObject(const GLObject& obj) {
    vertices = obj.vertices;
    modelMatrix = obj.modelMatrix;
    transfromedVertices = obj.transfromedVertices;
  }

  friend class GLScene;

  virtual GLObject* clone() = 0;

  Vertices& getVertices() { return vertices; }
  void setVertices(Vertices& vertices) { this->vertices = vertices; }
  Eigen::Matrix4d& getModelMatrix() { return modelMatrix; }
  void setModelMatrix(Eigen::Matrix4d& modelMatrix) { this->modelMatrix = modelMatrix; }
  Vertices& getTransformedVertices() { return transfromedVertices; }

  void pushVertice(double x, double y, double z) {
    Vertice v(x, y, z, 1);
    vertices.conservativeResize(vertices.rows() + 1, vertices.cols());
    vertices.row(vertices.rows() - 1) = v;
  }
  virtual void rotate_x(double a) { this->vertices = AffineUtils::rotate_x(this->vertices, a); }
  virtual void rotate_y(double a) { this->vertices = AffineUtils::rotate_y(this->vertices, a); }
  virtual void rotate_z(double a) { this->vertices = AffineUtils::rotate_z(this->vertices, a); }
  virtual void translate(double x, double y, double z) {
    this->vertices = AffineUtils::translate(this->vertices, x, y, z);
  }
  virtual void scale(double x, double y, double z) {
    this->vertices = AffineUtils::scale(this->vertices, x, y, z);
  }
  virtual void prepareTransform() = 0;
  virtual void transformVerticesWithMatrix(Eigen::Matrix4d& mtx) = 0;
  virtual void transformWithModelMatrix() = 0;
  virtual void draw(QPainter& painter) = 0;
  virtual void rasterize(GLScene& scene) = 0;
};

class GLMesh;

class GLMeshGroup : public GLObject {
 protected:
  GLMesh* parent;
  std::string name;
  Indices3 indices;
  NormIndices normIndices;
  std::vector<std::vector<Color01>> colors;
  std::vector<TexRef> texrefs;

 public:
  GLMeshGroup(GLMesh* parent, std::string& name) {
    this->parent = parent;
    this->name = name;
  }
  ~GLMeshGroup() = default;

  GLMesh* getParent() { return parent; }
  void setParent(GLMesh* parent) { this->parent = parent; }
  Indices3& getIndices() { return indices; }
  void setIndices(Indices3& indices) { this->indices = indices; }
  NormIndices& getNormIndices() { return normIndices; }
  void setNormIndices(NormIndices& normIndices) { this->normIndices = normIndices; }
  std::vector<std::vector<Color01>>& getColors() { return colors; }
  void setColors(std::vector<std::vector<Color01>>& colors) { this->colors = colors; }
  std::vector<TexRef>& getTexRefs() { return texrefs; }
  void setTexRefs(std::vector<TexRef>& texrefs) { this->texrefs = texrefs; }

  void addIndex3(Index3 idx);

  void addIndex3(Index3 idx, Color01 clr0, Color01 clr1, Color01 clr2) {
    indices.conservativeResize(indices.rows() + 1, indices.cols());
    indices.row(indices.rows() - 1) = idx;
    std::vector<Color01> tricolor{clr0, clr1, clr2};
    colors.push_back(std::move(tricolor));
  }

  void addNormIndex(NormIndex idx) {
    normIndices.conservativeResize(normIndices.rows() + 1, normIndices.cols());
    normIndices.row(normIndices.rows() - 1) = idx;
  }

  GLObject* clone() {
    GLMeshGroup* g = new GLMeshGroup(this->parent, this->name);
    g->indices = indices;
    g->normIndices = normIndices;
    g->colors = colors;
    g->texrefs = texrefs;
    return g;
  }
  void draw(QPainter& painter) {
    // TODO
  }

  void prepareTransform() {
    // TODO
  }

  void transformVerticesWithMatrix(Eigen::Matrix4d& mtx) {
    // TODO
  }
  void transformWithModelMatrix() {
    // TODO
  }

  void rasterize(GLScene& scene);

  void rasterizeTriangle(GLScene& scene, Triangle2& t, std::vector<Color01>& clrs,
                         GLMaterial* material);

  void drawSkeleton(QPainter& painter) {
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
};

class GLMesh : public GLObject {
 protected:
  Normals normals;
  Normals transfromedNormals;
  TexCoords texcoords;
  std::map<std::string, GLMeshGroup*> groups;
  std::map<std::string, GLMaterial*> materials;

 public:
  const static Color01 defaultColor;
  const static std::string defaultGroup;

  GLMesh() = default;
  ~GLMesh() {
    for (auto g : groups) {
      delete g.second;
    }
    for (auto m : materials) {
      delete m.second;
    }
  };
  GLMesh(const GLMesh& mesh) : GLObject(mesh) {
    groups = mesh.groups;
    normals = mesh.normals;
    transfromedNormals = mesh.transfromedNormals;
    texcoords = mesh.texcoords;
    // textures = mesh.textures;
    materials = mesh.materials;  // TODO: deepcopy?
  }
  GLObject* clone() {
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
    return p;
  }

  GLMeshGroup* getGroup(std::string& name) {
    if (groups.count(name)) {
      return groups[name];
    } else {
      groups[name] = new GLMeshGroup(this, name);
      return groups[name];
    }
  }
  Normals& getNormals() { return normals; }
  Normals& getTransformedNormals() { return transfromedNormals; }
  TexCoords& getTexCoords() { return texcoords; }
  GLMaterial* getMaterial(std::string name) {
    if (materials.count(name)) {
      return materials[name];
    } else {
      return nullptr;
    }
  }

  void addIndex3(Index3 idx) { addIndex3(const_cast<std::string&>(defaultGroup), idx); }
  void addIndex3(Index3 idx, Color01 clr0, Color01 clr1, Color01 clr2) {
    addIndex3(const_cast<std::string&>(defaultGroup), idx, clr0, clr1, clr2);
  }
  void addIndex3(std::string& groupName, Index3 idx) {
    getGroup(groupName)->addIndex3(idx, GLMesh::defaultColor, GLMesh::defaultColor,
                                   GLMesh::defaultColor);
  }

  void addIndex3(std::string& groupName, Index3 idx, Color01 clr0, Color01 clr1, Color01 clr2) {
    GLMeshGroup* group = getGroup(groupName);
    Indices3& indices = group->getIndices();
    indices.conservativeResize(indices.rows() + 1, indices.cols());
    indices.row(indices.rows() - 1) = idx;
    std::vector<Color01> tricolor{clr0, clr1, clr2};
    group->getColors().push_back(std::move(tricolor));
  }

  void pushNormal(double a, double b, double c) {
    Normal n(a, b, c);
    normals.conservativeResize(normals.rows() + 1, normals.cols());
    normals.row(normals.rows() - 1) = n;
  }

  void addNormIndex(std::string& groupName, NormIndex idx) {
    GLMeshGroup* group = getGroup(groupName);
    group->addNormIndex(idx);
  }

  void pushTexCoord(double u, double v) {
    TexCoord t(u, v);
    texcoords.conservativeResize(texcoords.rows() + 1, texcoords.cols());
    texcoords.row(texcoords.rows() - 1) = t;
  }

  void rotate_x(double a) {
    this->vertices = AffineUtils::rotate_x(this->vertices, a);
    this->normals = AffineUtils::normal_rotate_x(this->normals, a);
  }
  void rotate_y(double a) {
    this->vertices = AffineUtils::rotate_y(this->vertices, a);
    this->normals = AffineUtils::normal_rotate_y(this->normals, a);
  }
  void rotate_z(double a) {
    this->vertices = AffineUtils::rotate_z(this->vertices, a);
    this->normals = AffineUtils::normal_rotate_z(this->normals, a);
  }
  void translate(double x, double y, double z) {
    this->vertices = AffineUtils::translate(this->vertices, x, y, z);
    this->normals = AffineUtils::normal_translate(this->normals, x, y, z);
  }
  void scale(double x, double y, double z) {
    this->vertices = AffineUtils::scale(this->vertices, x, y, z);
    this->normals = AffineUtils::norm_scale(this->normals, x, y, z);
  }

  void transform() {
    this->transfromedVertices = AffineUtils::affine(this->vertices, this->modelMatrix);
    Eigen::Matrix3d m = this->modelMatrix.block(0, 0, 3, 3);
    this->transfromedNormals = AffineUtils::norm_affine(this->normals, m);
  }

  void prepareTransform() {
    this->transfromedVertices = this->vertices;
    this->transfromedNormals = this->normals;
  }

  void transformVerticesWithMatrix(Eigen::Matrix4d& mtx) {
    this->transfromedVertices = AffineUtils::affine(this->transfromedVertices, mtx);
  }
  void transformWithModelMatrix() {
    this->transfromedVertices = AffineUtils::affine(this->transfromedVertices, this->modelMatrix);
    Eigen::Matrix3d m = this->modelMatrix.block(0, 0, 3, 3);
    this->transfromedNormals = AffineUtils::norm_affine(this->transfromedNormals, m);
  }

  static GLMesh* readFromObjFile(std::string fpath);

  static GLMesh* fromObjModel(ObjModel* model);

  void rasterize(GLScene& scene);

  void draw(QPainter& painter) {
    // rasterize(painter);
    // drawSkeleton(painter);
  }
};

}  // namespace qtgl