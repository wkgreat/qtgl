#include "../scene.hpp"
#include <iostream>
int main() {
  qtgl::GLScene scene;

  Eigen::Matrix4d projmtx = scene.getProjection().projMatrix();
  Eigen::Matrix4d viewmtx = scene.getCamera().viewMatrix();
  Eigen::Matrix4d viewportmtx = scene.viewportMatrix();

  Eigen::Matrix4d transformMtx = viewmtx * projmtx * viewportmtx;
  Eigen::Matrix4d invTransformMtx = transformMtx.inverse();

  qtgl::Vertice v0(117, 32, 10, 1);
  std::cout << v0 << std::endl;
  qtgl::Vertice v1 = v0.transpose() * transformMtx;
  std::cout << v1 << std::endl;
  v1 /= v1[3];
  std::cout << v1 << std::endl;
  qtgl::Vertice v2 = v1.transpose() * invTransformMtx;
  std::cout << v2 << std::endl;
  v2 /= v2[3];
  std::cout << v2 << std::endl;

  return 0;
}