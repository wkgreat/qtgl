#pragma once

#include <QPainter>
#include <vector>
#include "camera.hpp"
#include "event.hpp"
#include "material.hpp"
#include "mesh.hpp"
#include "projection.hpp"
#include "shader.hpp"
#include "shadow.hpp"

namespace qtgl {

class GLObject;
class GLSceneConfiguration;

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

  std::vector<GLShadowMapping*> shadows;

  Eigen::Matrix4d transformMatrix;
  Eigen::Matrix4d invTransformMatrix;

  GLEventBus eventBus;
  GLSceneConfiguration* configuration;

 public:
  GLScene(double viewHeight = 768.0, double viewWidth = 1024.0);
  ~GLScene();
  GLSceneConfiguration* getConfiguration();
  GLCamera& getCamera();
  Fragments& getFragments();
  GLProjection& getProjection();
  void setViewHeight(double h);
  void setViewWidth(double w);
  void setViewSize(double w, double h);
  double getViewHeight();
  double getViewWidth();

  GLShader* getShader(IlluminationModel model);

  void setAmbient(Color01 ambient);
  Color01 getAmbient() const;

  void addObj(GLObject* obj);
  void addLight(GLLight* lgt);
  std::vector<GLLight*>& getLights();  // 光源-阴影同步
  std::vector<GLObject*>& getObjs();   // 对象-阴影同步
  std::vector<GLShadowMapping*>& getShadows();

  Eigen::Matrix4d viewportMatrix();

  void calculateTransformMatrix();

  // screen coordinator back to world coordinator
  Vertice screenVerticeBackToWorldVertice(double x, double y, double z, double w);

  Vertice screenVerticeBackToWorldVertice(Vertice v);

  Vertice screenVerticeBackToCameraVertice(Vertice v);

  void meshTransformToScreen(GLObject* obj);

  Fragments initFragmentsBuffer();
  void rasterize();

  void draw(QPainter& painter);

  bool isTriangleBack(Vertice& rp0, Vertice& rp1, Vertice& rp2, Normal& n0, Normal& n1, Normal& n2);
};

class GLSceneConfiguration {
 private:
  GLScene* scene;

  InterpolateMethod interpolateMethod = InterpolateMethod::BILINEAR;
  TexCoordType texCoordType = TexCoordType::PERSPECTIVE_CORRECT;

 public:
  GLSceneConfiguration(GLScene* scene) : scene(scene) {
    setInterpolateMethod(InterpolateMethod::BILINEAR);
  }
  ~GLSceneConfiguration() {}

  void setInterpolateMethod(InterpolateMethod method);
  InterpolateMethod getInterpolateMethod();

  void setTexCoordType(TexCoordType typ);
  TexCoordType getTexCoordType();
};

}  // namespace qtgl