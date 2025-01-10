#pragma once

#include <QString>
#include <QStringList>
#include <fstream>
#include <iostream>
#include <list>
#include <map>
#include <string>
#include "define.hpp"
#include "material.hpp"
#include "texture.hpp"

namespace qtgl {

class ObjMaterial {
 public:
  std::string dirpath;
  std::string name;
  double ns;
  Color01 ka;
  Color01 kd;
  Color01 ks;
  Color01 ke;
  double ni;
  double d;
  int illum;
  std::string map_ka = "";
  std::string map_kd = "";
  std::string map_refl = "";

  ObjMaterial() = default;
  ObjMaterial(std::string& dirpath, std::string& name) {
    this->dirpath = dirpath;
    this->name = name;
  };

  GLMaterial* toGLMaterial() {
    GLMaterial* material = new GLMaterial();
    material->setAmbient(ka);
    material->setDiffuse(kd);
    material->setSpecular(ks);
    material->setEmmisive(ke);
    material->setSpecularHighlight(ns);
    material->setOpticalDensity(ni);
    material->setDissolve(d);
    if (illum == 0) {
      material->setIllumination(IlluminationModel::CONSTANT);
    } else if (illum == 1) {
      material->setIllumination(IlluminationModel::LAMBERTIAN);
    } else if (illum == 2) {
      material->setIllumination(IlluminationModel::LAMBERTIAN_BLINN_PHONG);
    } else {
      material->setIllumination(IlluminationModel::LAMBERTIAN_BLINN_PHONG);
    }
    if (!map_ka.empty()) {
      GLTexture* t = new InterpolateGLTexture(dirpath + "/" + map_ka);
      material->setAmbientTexture(t);
    } else {
      material->setAmbientTexture(nullptr);
    }
    if (!map_kd.empty()) {
      GLTexture* t = new InterpolateGLTexture(dirpath + "/" + map_kd);
      material->setDiffuseTexture(t);
    } else {
      material->setDiffuseTexture(nullptr);
    }
    return material;
  }
};

class ObjMaterialLib {
 public:
  std::string dirpath;
  std::string libname;
  std::map<std::string, ObjMaterial> mtls;
  ObjMaterialLib() = default;
  ObjMaterialLib(std::string& dirpath, std::string& libname) {
    this->dirpath = dirpath;
    this->libname = libname;
  }
  static ObjMaterialLib* loadMtlLib(std::string& dirpath, std::string& libname);
};

class TexRef {
 public:
  std::string mtlname = "";
  Eigen::Vector3i indices;
  TexRef() = default;
  TexRef(std::string& mtlname, Eigen::Vector3i& indices) {
    this->mtlname = mtlname;
    this->indices = indices;
  }
};

class ObjModel;

class ObjModelGroup {
 public:
  std::string name;
  ObjModel* parent;
  Indices3 indices;
  NormIndices normIndices;
  std::string mtlname;
  std::vector<TexRef> texrefs;

  ObjModelGroup() = default;

  ObjModelGroup(ObjModel* parent, std::string& name) {
    this->parent = parent;
    this->name = name;
  }

  void addIndex3(Index3 idx) {
    indices.conservativeResize(indices.rows() + 1, indices.cols());
    indices.row(indices.rows() - 1) = idx;
  }

  void addNormIndex(NormIndex idx) {
    normIndices.conservativeResize(normIndices.rows() + 1, normIndices.cols());
    normIndices.row(normIndices.rows() - 1) = idx;
  }

  void addTexRef(TexRef& texref) { texrefs.push_back(texref); }
};

class ObjModel {
 public:
  const static std::string defaultGroup;
  std::string objpath;
  std::string dirpath;
  std::string objname;
  ObjMaterialLib* mtllib;
  std::map<std::string, ObjModelGroup> groups;
  Vertices vertices;
  Normals normals;
  TexCoords texcoords;

  ObjModel() = default;
  ~ObjModel() { delete mtllib; };

  void pushVertice(double x, double y, double z) {
    Vertice v(x, y, z, 1);
    vertices.conservativeResize(vertices.rows() + 1, vertices.cols());
    vertices.row(vertices.rows() - 1) = v;
  }

  void pushNormal(double a, double b, double c) {
    Normal n(a, b, c);
    normals.conservativeResize(normals.rows() + 1, normals.cols());
    normals.row(normals.rows() - 1) = n;
  }

  void pushTexCoord(double u, double v) {
    TexCoord t(u, v);
    texcoords.conservativeResize(texcoords.rows() + 1, texcoords.cols());
    texcoords.row(texcoords.rows() - 1) = t;
  }

  void setVertices(std::list<double>& vs) {
    size_t rows = vs.size() / 3;
    vertices.conservativeResize(rows, 4);
    int r = 0;
    auto p = vs.begin();
    Vertice v;
    while (p != vs.end()) {
      v[0] = *p++;
      v[1] = *p++;
      v[2] = *p++;
      v[3] = 1;
      vertices.row(r++) = v;
    }
  }
  void setNormals(std::list<double>& ns) {
    size_t rows = ns.size() / 3;
    normals.conservativeResize(rows, 3);
    int r = 0;
    auto p = ns.begin();
    Normal n;
    while (p != ns.end()) {
      n[0] = *p++;
      n[1] = *p++;
      n[2] = *p++;
      normals.row(r++) = n;
    }
  }
  void setTexcoords(std::list<double>& ts) {
    size_t rows = ts.size() / 2;
    texcoords.conservativeResize(rows, 2);
    int r = 0;
    auto p = ts.begin();
    TexCoord t;
    while (p != ts.end()) {
      t[0] = *p++;
      t[1] = *p++;
      texcoords.row(r++) = t;
    }
  }

  ObjModelGroup& getGroup(std::string& name) {
    if (groups.count(name)) {
      return groups[name];
    } else {
      groups[name] = ObjModelGroup(this, name);
      return groups[name];
    }
  }

  void addIndex3(std::string& groupName, Index3 idx) { getGroup(groupName).addIndex3(idx); }

  void addNormIndex(std::string& groupName, NormIndex idx) {
    getGroup(groupName).addNormIndex(idx);
  }

  void addTexRef(std::string& groupName, std::string& mtlname, Eigen::Vector3i& indices) {
    TexRef texref;
    texref.mtlname = mtlname;
    texref.indices = indices;
    getGroup(groupName).texrefs.push_back(texref);
  }

  static ObjModel* loadObj(const std::string& objpath);
};

}  // namespace qtgl