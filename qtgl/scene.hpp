#pragma once

#include <QPainter>
#include "camera.hpp"
#include "material.hpp"
#include "projection.hpp"
#include "shader.hpp"

namespace qtgl {

class GLObject;

class GLScene {
 private:
  double viewHeight;
  double viewWidth;
  GLCamera camera;
  GLProjection projection;
  std::vector<GLObject*> objs;
  std::vector<GLLight*> lights;
  Fragments fragments;
  std::map<IlluminationModel, GLShader*> shadermap;
  Color01 ambient = {1, 1, 1, 1};

  Eigen::Matrix4d transformMatrix;
  Eigen::Matrix4d invTransformMatrix;

 public:
  GLScene(double viewHeight = 768.0, double viewWidth = 1024.0)
      : viewHeight(viewHeight), viewWidth(viewWidth) {
    this->projection.height = viewHeight;
    this->projection.width = viewWidth;
    this->shadermap[IlluminationModel::LAMBERTIAN] = new LambertianGLShader();
    this->shadermap[IlluminationModel::LAMBERTIAN_BLINN_PHONG] = new LambertialBlinnPhongGLShader();
  }
  ~GLScene();

  GLCamera& getCamera() { return this->camera; }
  Fragments& getFragments() { return this->fragments; }
  GLProjection& getProjection() { return this->projection; }
  void setViewHeight(double h) {
    this->viewHeight = h;
    this->projection.height = h;
  }
  void setViewWidth(double w) {
    this->viewWidth = w;
    this->projection.width = w;
  }
  void setViewSize(double w, double h) {
    this->setViewWidth(w);
    this->setViewHeight(h);
  }
  GLShader* getShader(IlluminationModel model) { return this->shadermap[model]; }

  void setAmbient(Color01 ambient) { this->ambient = ambient; }
  Color01 getAmbient() const { return this->ambient; }

  void addObj(GLObject* obj) { objs.push_back(obj); }
  void addLight(GLLight* lgt) { lights.push_back(lgt); }
  std::vector<GLLight*>& getLights() { return this->lights; }
  std::vector<GLObject*>& getObjs() { return this->objs; }

  Eigen::Matrix4d viewportMatrix() {
    double hw = this->viewWidth / 2;
    double hh = this->viewHeight / 2;
    Eigen::Matrix4d viewMtx;
    viewMtx << hw, 0, 0, 0,  //
        0, hh, 0, 0,         //
        0, 0, 1, 0,          //
        hw, hh, 0, 1;        //
    return viewMtx;
  }

  void calculateTransformMatrix() {
    // 视图变换矩阵 * 投影变换矩阵 * 视口变换矩阵
    transformMatrix = camera.viewMatrix() * projection.projMatrix() * viewportMatrix();
    // 逆变换矩阵 用于将屏幕坐标转换为世界坐标
    invTransformMatrix = transformMatrix.inverse();
  }

  // screen coordinator back to world coordinator
  Vertice screenVerticeBackToWorldVertice(double x, double y, double z, double w) {
    return screenVerticeBackToWorldVertice({x, y, z, w});
  }

  Vertice screenVerticeBackToWorldVertice(Vertice v) {
    v = v.transpose() * invTransformMatrix;
    v /= v[3];
    return v;
  }

  Vertice screenVerticeBackToCameraVertice(Vertice v) {
    v = v.transpose() * invTransformMatrix;
    v /= v[3];
    v = v.transpose() * this->getCamera().viewMatrix();
    return v;
  }

  void meshTransformToScreen(GLObject* obj);

  Fragments initFragmentsBuffer() {
    Fragments fs(this->viewHeight, std::vector<Fragment>(this->viewWidth, Fragment::init()));
    return fs;
  }

  void draw(QPainter& painter);

  bool isTriangleBack(Vertice& rp0, Vertice& rp1, Vertice& rp2, Normal& n0, Normal& n1,
                      Normal& n2) {
    Eigen::Vector3d p = ((rp0 + rp1 + rp2) / 3).head(3);
    Eigen::Vector3d n = (n0 + n1 + n2).head(3).normalized();
    Eigen::Vector3d c = this->getCamera().getPositionVertice().head(3);
    Eigen::Vector3d v1 = n;
    Eigen::Vector3d v2 = (c - p).head(3).normalized();
    return v1.dot(v2) < 0;
  }
};
}  // namespace qtgl