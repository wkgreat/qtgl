#pragma once

#include <map>
#include <vector>
#include "affineutils.hpp"
#include "define.hpp"
#include "material.hpp"

namespace qtgl {

enum class MeshType {
  POINTS,
  LINES,
  LINE_LOOP,
  LINE_STRIP,
  TRIANGLES,
  TRIANGLE_STRIP,
  TRIANGLE_FAN
};

class GLPrimitive {
 private:
  MeshType meshType;
  Vertices worldPos;
  Vertices screenPos;
  Normals normal;
  std::map<int, TexCoords> texcoords;
  Indices3 indices;
  GLMaterialBase* material;
  Eigen::Matrix4d matrix;
  Eigen::Matrix4d invMatrix;

 public:
  GLPrimitive() {}
  GLPrimitive(MeshType type, Indices3& indices, Vertices& worldPos, Normals& normal,
              GLMaterialBase* material)
      : meshType(type), indices(indices), worldPos(worldPos), normal(normal), material(material) {}

  void addTexCoord(int k, TexCoords& cs) { texcoords[k] = cs; }

  void transformFromWorldToScreen(Eigen::Matrix4d& matrix) {
    this->matrix = matrix;
    this->invMatrix = matrix.inverse();
    screenPos = AffineUtils::affine(worldPos, matrix);
  }

  GLMaterialBase* getMaterial() { return this->material; }
  Eigen::Matrix4d& getMatrix() { return this->matrix; }
  Eigen::Matrix4d& getInvMatrix() { return this->invMatrix; }
  Vertices& getWorldPos() { return this->worldPos; }
  Indices3& getIndices() { return this->indices; }

  // TODO iterator
  std::vector<Triangle3> getTriangles() {
    int n = indices.rows();
    std::vector<Triangle3> triangles;
    if (meshType == MeshType::TRIANGLES) {
      for (int i = 0; i < n; ++i) {
        Index3 idx = indices.row(i);
        Vertice wp0 = worldPos.row(idx[0]);
        Vertice wp1 = worldPos.row(idx[1]);
        Vertice wp2 = worldPos.row(idx[2]);
        Vertice sp0 = screenPos.row(idx[0]);
        Vertice sp1 = screenPos.row(idx[1]);
        Vertice sp2 = screenPos.row(idx[2]);
        Normal n0 = normal.row(idx[0]).normalized();
        Normal n1 = normal.row(idx[1]).normalized();
        Normal n2 = normal.row(idx[2]).normalized();
        Triangle3 t(sp0, sp1, sp2, n0, n1, n2);
        t.setWorldPos(wp0, wp1, wp2);

        for (auto it = texcoords.begin(); it != texcoords.end(); ++it) {
          int k = it->first;
          TexCoord c0 = (it->second).row(idx[0]);
          TexCoord c1 = (it->second).row(idx[1]);
          TexCoord c2 = (it->second).row(idx[2]);
          t.addTexCoords(k, c0, c1, c2);
        }

        triangles.push_back(t);
      }
    } else {
      // TODO other types primitives
    }
    return triangles;
  }
};

};  // namespace qtgl