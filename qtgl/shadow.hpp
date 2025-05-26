#pragma once

#include <limits>
#include "camera.hpp"
#include "define.hpp"
#include "event.hpp"
#include "projection.hpp"
#include "shader.hpp"

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

  GLEventBus* eventBus;

  GLShadowMapping(GLScene* scene, GLLight* lgt);
  ~GLShadowMapping();
  void refreshDepthMap();
  Eigen::Matrix4d viewportMatrix();

  void rasterize();
  void rasterizeMeshGroup(GLMeshGroup* g);
  void rasterizeTriangle(Vertice p0, Vertice p1, Vertice p2);

  bool isWorldPosInShadow(Vertice p);

  void setEventBus(GLEventBus* eventBus) { this->eventBus = eventBus; }
  GLEventBus* getEventBus() { return this->eventBus; }
};

}  // namespace qtgl