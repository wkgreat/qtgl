#pragma once

#include "define.hpp"
#include "mathutils.hpp"

namespace qtgl {
enum class GLProjectionMode {
  ORTHOGRAPHIC,  // 正射投影
  PRESPECTIVE    // 透视投影
};

class GLProjection {
 private:
  double height;
  double width;
  double _near;
  double _far;
  double hfov = MathUtils::PI / 3;
  GLProjectionMode mode;

 public:
  GLProjection()
      : height(768), width(1024), _near(1), _far(2000), mode(GLProjectionMode::PRESPECTIVE) {}
  GLProjection(double height, double width, double near, double far, GLProjectionMode mode)
      : height(height), width(width), _near(near), _far(far), mode(mode) {}
  Eigen::Matrix4d orthographicProjMatrix() {
    double vfov = hfov * (height / width);
    double right = tan(hfov / 2) * _near;
    double left = -right;
    double top = tan(vfov / 2) * _near;
    double bottom = -top;

    double m00 = 2 / (right - left);
    double m11 = 2 / (top - bottom);
    double m22 = 2 / (_near - _far);
    double m30 = -(right + left) / (right - left);
    double m31 = -(top + bottom) / (top - bottom);
    double m32 = -(_near + _far) / (_near - _far);

    Eigen::Matrix4d projMtx;
    projMtx << m00, 0, 0, 0,  //
        0, m11, 0, 0,         //
        0, 0, m22, 0,         //
        m30, m31, m32, 1;
    return projMtx;
  }
  Eigen::Matrix4d perspectiveProjMatrix() {
    double vfov = hfov * (height / width);
    double right = tan(hfov / 2) * _near;
    double left = -right;
    double top = tan(vfov / 2) * _near;
    double bottom = -top;
    double m00 = 2 / (right - left);
    double m11 = 2 / (top - bottom);
    double m22 = (_far + _near) / (_far - _near);
    double m32 = -2 * _near * _far / (_far - _near);
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
  double getNear() const { return _near; }
  double getFar() const { return _far; }
  double getHfov() const { return hfov; }
  double getAspect() const { return height / width; }
  GLProjectionMode getMode() const { return mode; }
  void setHeight(double height) { this->height = height; }
  void setWidth(double width) { this->width = width; }
  void setMode(GLProjectionMode mode) { this->mode = mode; }
};
}  // namespace qtgl