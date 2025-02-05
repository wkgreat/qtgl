#pragma once

#include "define.hpp"

#include "shader.hpp"

#include <limits>

#include "projection.hpp"

#include "camera.hpp"

namespace qtgl {
class GLScene;
class GLMeshGroup;

class GLShadowMapping {
 public:
  GLScene* scene;
  PointGLLight* lgt;
  constexpr static double DEPTH_INF = std::numeric_limits<double>::max() / 2;
  int mapHeight;
  int mapWidth;
  std::vector<std::vector<double>>* depthMap = nullptr;
  double bias = 10E-8;

  GLProjection projection;
  GLCamera camera;
  Eigen::Matrix4d transformMatrix;
  Eigen::Matrix4d invTransformMatrix;

  GLShadowMapping(GLScene* scene, GLLight* lgt);
  ~GLShadowMapping();
  void refreshDepthMap();
  Eigen::Matrix4d viewportMatrix();

  void rasterize();
  void rasterizeMeshGroup(GLMeshGroup* g);
  void rasterizeTriangle(Vertice p0, Vertice p1, Vertice p2);

  bool isWorldPosInShadow(Vertice p);
};

}  // namespace qtgl