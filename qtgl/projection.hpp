#pragma once

#include "define.hpp"
#include "mathutils.hpp"

namespace qtgl {
enum class GLProjectionMode {
  ORTHOGRAPHIC,  // 正射投影
  PRESPECTIVE    // 透视投影
};

class GLProjection {
 public:
  double height;
  double width;
  double near;
  double far;
  double hfov = MathUtils::PI / 3;
  GLProjectionMode mode;
  GLProjection()
      : height(768), width(1024), near(1), far(2000), mode(GLProjectionMode::PRESPECTIVE) {}
  GLProjection(double height, double width, double near, double far, GLProjectionMode mode)
      : height(height), width(width), near(near), far(far), mode(mode) {}
  Eigen::Matrix4d orthographicProjMatrix() {
    double vfov = hfov * (height / width);
    double right = tan(hfov / 2) * near;
    double left = -right;
    double top = tan(vfov / 2) * near;
    double bottom = -top;

    double m00 = 2 / (right - left);
    double m11 = 2 / (top - bottom);
    double m22 = 2 / (near - far);
    double m30 = -(right + left) / (right - left);
    double m31 = -(top + bottom) / (top - bottom);
    double m32 = -(near + far) / (near - far);

    Eigen::Matrix4d projMtx;
    projMtx << m00, 0, 0, 0,  //
        0, m11, 0, 0,         //
        0, 0, m22, 0,         //
        m30, m31, m32, 1;
    return projMtx;
  }
  Eigen::Matrix4d perspectiveProjMatrix() {
    double vfov = hfov * (height / width);
    double right = tan(hfov / 2) * near;
    double left = -right;
    double top = tan(vfov / 2) * near;
    double bottom = -top;
    double m00 = 2 / (right - left);
    double m11 = 2 / (top - bottom);
    double m22 = (far + near) / (far - near);
    double m32 = -2 * near * far / (far - near);
    Eigen::Matrix4d projMtx;
    projMtx << m00, 0, 0, 0,  //
        0, m11, 0, 0,         //
        0, 0, m22, 1,         //
        0, 0, m32, 0;
    return projMtx;
  }
  Eigen::Matrix4d projMatrix() {
    if (mode == GLProjectionMode::ORTHOGRAPHIC) {
      return orthographicProjMatrix();
    } else {
      return perspectiveProjMatrix();
    }
  }
};
}  // namespace qtgl