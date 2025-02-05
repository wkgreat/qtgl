#include "shadow.hpp"
#include "mesh.hpp"
#include "scene.hpp"

namespace qtgl {

GLShadowMapping::GLShadowMapping(GLScene* scene, GLLight* lgt) {
  this->scene = scene;
  this->lgt = dynamic_cast<PointGLLight*>(lgt);
  if (this->lgt) {
    mapHeight = scene->getViewHeight();
    mapWidth = scene->getViewWidth();
    projection = scene->getProjection();
    projection.height = mapHeight;
    projection.width = mapWidth;
    projection.mode = GLProjectionMode::PRESPECTIVE;
    camera.lookAt(this->lgt->getPosition()[0], this->lgt->getPosition()[1],
                  this->lgt->getPosition()[2], 0, 0, 0);
    refreshDepthMap();
  }
}
GLShadowMapping::~GLShadowMapping() {
  if (depthMap) {
    delete depthMap;
    depthMap = nullptr;
  }
}
void GLShadowMapping::refreshDepthMap() {
  if (this->depthMap) {
    delete depthMap;
    this->depthMap = nullptr;
  }
  this->depthMap =
      new std::vector<std::vector<double>>(mapHeight, std::vector<double>(mapWidth, DEPTH_INF));

  // 视图变换矩阵 * 投影变换矩阵 * 视口变换矩阵
  transformMatrix = camera.viewMatrix() * projection.projMatrix() * viewportMatrix();
  // 逆变换矩阵 用于将屏幕坐标转换为世界坐标
  invTransformMatrix = transformMatrix.inverse();

  for (GLObject* obj : this->scene->getObjs()) {
    obj->transformVerticesToScreen(transformMatrix);
  }

  this->rasterize();
}

Eigen::Matrix4d GLShadowMapping::viewportMatrix() {
  double hw = this->mapWidth / 2;
  double hh = this->mapHeight / 2;
  Eigen::Matrix4d viewMtx;
  viewMtx << hw, 0, 0, 0,  //
      0, hh, 0, 0,         //
      0, 0, 1, 0,          //
      hw, hh, 0, 1;        //
  return viewMtx;
}

void GLShadowMapping::rasterize() {
  for (GLObject* obj : this->scene->getObjs()) {
    GLMesh* mesh = dynamic_cast<GLMesh*>(obj);
    if (mesh) {
      for (auto p : mesh->getGroups()) {
        GLMeshGroup* g = p.second;
        rasterizeMeshGroup(g);
      }
    }
  }
}

void GLShadowMapping::rasterizeMeshGroup(GLMeshGroup* g) {
  Indices3 indices = g->getIndices();
  int n = indices.rows();
  for (int i = 0; i < n; ++i) {
    Index3 idx = indices.row(i);
    Vertice rp0 = g->getParent()->getVertices().row(idx[0]);
    Vertice rp1 = g->getParent()->getVertices().row(idx[1]);
    Vertice rp2 = g->getParent()->getVertices().row(idx[2]);
    Vertice p0 = g->getParent()->getScreenVertices().row(idx[0]);
    Vertice p1 = g->getParent()->getScreenVertices().row(idx[1]);
    Vertice p2 = g->getParent()->getScreenVertices().row(idx[2]);

    // // 背面剔除
    // if (this->scene->isTriangleBack(rp0, rp1, rp2, n0, n1, n2)) {
    //   continue;
    // }
    p0 = p0 / p0[3];
    p1 = p1 / p1[3];
    p2 = p2 / p2[3];
    rasterizeTriangle(p0, p1, p2);
  }
}

void GLShadowMapping::rasterizeTriangle(Vertice p0, Vertice p1, Vertice p2) {
  Triangle t(p0, p1, p2);
  // mbr
  int xmin = static_cast<int>(std::min(std::min(t.x0(), t.x1()), t.x2()));
  int xmax = static_cast<int>(std::max(std::max(t.x0(), t.x1()), t.x2()));
  int ymin = static_cast<int>(std::min(std::min(t.y0(), t.y1()), t.y2()));
  int ymax = static_cast<int>(std::max(std::max(t.y0(), t.y1()), t.y2()));

  double depth;
  Triangle::BarycentricCoordnates coord;

  for (int x = xmin; x <= xmax; ++x) {
    if (x < 0 || x >= mapWidth) continue;
    for (int y = ymin; y <= ymax; ++y) {
      if (y < 0 || y >= mapHeight) continue;

      coord = t.resovleBarycentricCoordnates(x, y);

      if (coord.alpha >= 0 && coord.beta >= 0 && coord.gamma >= 0) {
        depth = coord.alpha * t.z0() + coord.beta * t.z1() + coord.gamma * t.z2();
        if (depth < (*depthMap)[y][x]) {  // double bias
          (*depthMap)[y][x] = depth;
        }
      }
    }
  }
}

bool GLShadowMapping::isWorldPosInShadow(Vertice p) {
  // return false;
  p = p.transpose() * transformMatrix;
  p = p / p[3];
  int x = static_cast<int>(p[0]);
  int y = static_cast<int>(p[1]);
  double d = p[2];
  if (y < 0 || y >= mapHeight || x < 0 || x >= mapWidth) {
    // std::cout << "isWorldPosInShadow out of depth map: " << x << "," << y << std::endl;
    return false;
  }
  if (MathUtils::equal<double>(d, (*depthMap)[y][x], bias)) {
    return false;
  } else {
    return true;
  }
}

}  // namespace qtgl