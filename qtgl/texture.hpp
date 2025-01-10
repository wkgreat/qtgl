#pragma once
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgcodecs.hpp>
#include "define.hpp"

namespace qtgl {

class GLTexture {
 public:
  GLTexture() = default;
  virtual ~GLTexture() = default;
  virtual Color01 sample(TexCoord& coord) = 0;

  static TexCoord interpolateTexCoord(Triangle2& t, double alpha, double beta, double gamma) {
    return prespectiveCorrectInterpolate(t, alpha, beta, gamma);
  }

  /*
  纹理坐标插值 - 基础版
  */
  static TexCoord basicInterpolcate(Triangle2& t, double alpha, double beta, double gamma) {
    return alpha * t.getTexCoord0() + beta * t.getTexCoord1() + gamma * t.getTexCoord2();
  }

  /*
  纹理坐标插值 - 透视正确版
  REF: [Fundamental of Computer Graphics] P255 11.2.4 Perspective Correct Interpolation
  */
  static TexCoord prespectiveCorrectInterpolate(Triangle2& t, double alpha, double beta,
                                                double gamma) {
    TexCoord& c0 = t.getTexCoord0();
    TexCoord& c1 = t.getTexCoord1();
    TexCoord& c2 = t.getTexCoord2();
    double w0 = t.w0();
    double w1 = t.w1();
    double w2 = t.w2();
    double ls = alpha * (1 / w0) + beta * (1 / w1) + gamma * (1 / w2);
    TexCoord c = (alpha * (c0 / w0) + beta * (c1 / w1) + gamma * (c2 / w2)) / ls;
    return c;
  }
};

class InterpolateGLTexture : public GLTexture {
 public:
  cv::Mat mat;
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

  // TODO 双线性插值
  Color01 sample(TexCoord& coord) {
    int i = round(mat.rows - coord[1] * mat.rows);
    int j = round(coord[0] * mat.cols);
    i = MAX(0, i);  // TODO 临时添加
    j = MAX(0, j);  // TODO 临时添加
    cv::Vec3b v = mat.at<cv::Vec3b>(i % mat.rows, j % mat.cols);
    Color01 c(v[0] / 255.0, v[1] / 255.0, v[2] / 255.0, 1);
    return c;
  }
};

}  // namespace qtgl