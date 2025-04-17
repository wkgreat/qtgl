#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgcodecs.hpp>
#include "define.hpp"

namespace qtgl {

enum class TexCoordType { BASIC, PERSPECTIVE_CORRECT };

class GLTexture {
 public:
  static TexCoordType texCoordType;
  GLTexture() = default;
  virtual ~GLTexture() = default;
  virtual Color01 sample(TexCoord& coord) = 0;

  static TexCoord interpolateTexCoord(Triangle3& t, int k, double alpha, double beta,
                                      double gamma) {
    switch (texCoordType) {
      case TexCoordType::BASIC:
        return basicInterpolate(t, k, alpha, beta, gamma);
      case TexCoordType::PERSPECTIVE_CORRECT:
        return prespectiveCorrectInterpolate(t, k, alpha, beta, gamma);
      default:
        return prespectiveCorrectInterpolate(t, k, alpha, beta, gamma);
    }
  }

  /*
  纹理坐标插值 - 基础版
  */
  static TexCoord basicInterpolate(Triangle3& t, int k, double alpha, double beta, double gamma) {
    return alpha * t.getTexCoord0(k) + beta * t.getTexCoord1(k) + gamma * t.getTexCoord2(k);
  }

  /*
  纹理坐标插值 - 透视正确版
  REF: [Fundamental of Computer Graphics] P255 11.2.4 Perspective Correct Interpolation
  */
  static TexCoord prespectiveCorrectInterpolate(Triangle3& t, int k, double alpha, double beta,
                                                double gamma) {
    TexCoord& c0 = t.getTexCoord0(k);
    TexCoord& c1 = t.getTexCoord1(k);
    TexCoord& c2 = t.getTexCoord2(k);
    double w0 = t.w0();
    double w1 = t.w1();
    double w2 = t.w2();
    double ls = alpha * (1 / w0) + beta * (1 / w1) + gamma * (1 / w2);
    TexCoord c = (alpha * (c0 / w0) + beta * (c1 / w1) + gamma * (c2 / w2)) / ls;
    return c;
  }
};

enum class InterpolateMethod { LIINEAR, BILINEAR };

class InterpolateGLTexture : public GLTexture {
 public:
  cv::Mat mat;
  InterpolateMethod interpolateMtd = InterpolateMethod::BILINEAR;
  InterpolateGLTexture() = default;
  ~InterpolateGLTexture() = default;
  InterpolateGLTexture(std::string mapref) {
    cv::Mat img = cv::imread(mapref);
    if (img.empty()) {
      std::cerr << "Error: cannot load image " << mapref << std::endl;
      return;
    }
    this->mat = img;
  };

  void setInterpolateMethod(InterpolateMethod mtd) { this->interpolateMtd = mtd; }
  InterpolateMethod getInterpolateMethod() { return this->interpolateMtd; }

  Color01 sample(TexCoord& coord) {
    switch (interpolateMtd) {
      case InterpolateMethod::LIINEAR:
        return sample_linear(coord);
      case InterpolateMethod::BILINEAR:
        return sample_bilinear(coord);
      default:
        return sample_bilinear(coord);
    };
  }

  Color01 colorAt(int u, int v) {
    while (u < 0) u += mat.cols;
    while (v < 0) v += mat.rows;
    cv::Vec3b c = mat.at<cv::Vec3b>(v % mat.rows, u % mat.cols);
    return {c[0] / 255.0, c[1] / 255.0, c[2] / 255.0, 1};
  }

  /*
  linear interplation
  Fundamental of Computer Graphics P251 11.2.2
  */
  Color01 sample_linear(TexCoord& coord) {
    int v = round(mat.rows - coord[1] * mat.rows);
    int u = round(coord[0] * mat.cols);
    return colorAt(u, v);
  }

  /*
  bilinear interplation
  Fundamental of Computer Graphics P264
  */
  Color01 sample_bilinear(TexCoord& coord) {
    int vp = mat.rows - coord[1] * mat.rows;
    int up = coord[0] * mat.cols;
    int iv0 = static_cast<int>(vp);
    int iu0 = static_cast<int>(up);
    int iv1 = iv0 + 1;
    int iu1 = iu0 + 1;
    double av = iv1 - vp;
    double au = iu1 - up;

    Color01 c00 = colorAt(iu0, iv0) * au * av;
    Color01 c01 = colorAt(iu0, iv1) * au * (1 - av);
    Color01 c10 = colorAt(iu1, iv0) * (1 - au) * av;
    Color01 c11 = colorAt(iu1, iv1) * (1 - au) * (1 - av);
    Color01 c = c00 + c01 + c10 + c11;
    return c;
  }
};

}  // namespace qtgl