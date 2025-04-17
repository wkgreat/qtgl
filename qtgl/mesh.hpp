#pragma once

#include <assert.h>
#include <QPainter>
#include <QString>
#include <QStringList>
#include <fstream>
#include <iostream>
#include <map>
#include "affineutils.hpp"
#include "material.hpp"
#include "objmodel.hpp"
#include "primitive.hpp"
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
  Vertices screenVertices;
  bool editing = false;

 public:
  GLObject() = default;
  virtual ~GLObject() = default;
  GLObject(const GLObject& obj) {
    vertices = obj.vertices;
    modelMatrix = obj.modelMatrix;
    screenVertices = obj.screenVertices;
    transfromedVertices = obj.transfromedVertices;
    editing = obj.editing;
  }

  friend class GLScene;

  virtual GLObject* clone() = 0;

  Vertices& getVertices() { return vertices; }
  void setVertices(Vertices& vertices) { this->vertices = vertices; }
  Eigen::Matrix4d& getModelMatrix() { return modelMatrix; }
  void setModelMatrix(Eigen::Matrix4d& modelMatrix) {
    assert(editing);
    this->modelMatrix = modelMatrix;
  }
  Vertices& getTransformedVertices() { return transfromedVertices; }
  Vertices& getScreenVertices() { return screenVertices; }

  void pushVertice(double x, double y, double z) {
    assert(editing);
    Vertice v(x, y, z, 1);
    vertices.conservativeResize(vertices.rows() + 1, vertices.cols());
    vertices.row(vertices.rows() - 1) = v;
  }
  virtual void transformVerticesToScreen(Eigen::Matrix4d& mtx) = 0;
  virtual void transformWithModelMatrix() = 0;
  virtual void draw(QPainter& painter) = 0;
  virtual void rasterize(GLScene& scene) = 0;
  virtual void startEditing() { editing = true; }
  virtual void finishEditing() {
    this->transformWithModelMatrix();
    editing = false;
  }
  virtual std::vector<GLPrimitive> getPrimitives() = 0;
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
  GLMeshGroup(GLMesh* parent, std::string& name);
  ~GLMeshGroup();

  GLMesh* getParent();
  void setParent(GLMesh* parent);
  Indices3& getIndices();
  void setIndices(Indices3& indices);
  NormIndices& getNormIndices();
  void setNormIndices(NormIndices& normIndices);
  std::vector<std::vector<Color01>>& getColors();
  void setColors(std::vector<std::vector<Color01>>& colors);
  std::vector<TexRef>& getTexRefs();
  void setTexRefs(std::vector<TexRef>& texrefs);

  void addIndex3(Index3 idx);
  void addIndex3(Index3 idx, Color01 clr0, Color01 clr1, Color01 clr2);

  void addNormIndex(NormIndex idx);

  GLObject* clone();
  void draw(QPainter& painter);

  void transformVerticesToScreen(Eigen::Matrix4d& mtx);
  void transformWithModelMatrix();

  void rasterize(GLScene& scene);
  void rasterizeTriangle(GLScene& scene, Triangle3& t, GLMaterial* material);

  void drawSkeleton(QPainter& painter);

  void startEditing();
  void finishEditing();

  std::vector<GLPrimitive> getPrimitives();
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
  ~GLMesh();
  GLMesh(const GLMesh& mesh);

  GLObject* clone();

  GLMeshGroup* getGroup(std::string& name);
  std::map<std::string, GLMeshGroup*>& getGroups();

  Normals& getNormals();
  Normals& getTransformedNormals();
  TexCoords& getTexCoords();
  GLMaterial* getMaterial(std::string name);

  std::map<std::string, GLMaterial*>& getMaterials();

  void addIndex3(Index3 idx);

  void addIndex3(Index3 idx, Color01 clr0, Color01 clr1, Color01 clr2);

  void addIndex3(std::string& groupName, Index3 idx);

  void addIndex3(std::string& groupName, Index3 idx, Color01 clr0, Color01 clr1, Color01 clr2);

  void pushNormal(double a, double b, double c);

  void addNormIndex(std::string& groupName, NormIndex idx);

  void pushTexCoord(double u, double v);

  void transformVerticesToScreen(Eigen::Matrix4d& mtx);

  void transformWithModelMatrix();

  static GLMesh* readFromObjFile(std::string fpath);

  static GLMesh* fromObjModel(ObjModel* model);

  void rasterize(GLScene& scene);

  void draw(QPainter& painter);

  void startEditing();

  void finishEditing();

  std::vector<GLPrimitive> getPrimitives();
};

}  // namespace qtgl