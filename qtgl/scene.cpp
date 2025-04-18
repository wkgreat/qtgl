#include "scene.hpp"
#include "mesh.hpp"

namespace qtgl {

GLScene::GLScene(double viewHeight, double viewWidth)
    : viewHeight(viewHeight), viewWidth(viewWidth) {
  this->projection.height = viewHeight;
  this->projection.width = viewWidth;
  this->shadermap[IlluminationModel::LAMBERTIAN] = new LambertianGLShader();
  this->shadermap[IlluminationModel::LAMBERTIAN_BLINN_PHONG] = new LambertialBlinnPhongGLShader();
  this->configuration = new GLSceneConfiguration(this);
  this->eventBus = new GLEventBus();
}

GLScene::~GLScene() {
  for (GLObject* obj : objs) {
    delete obj;
  }
  for (GLLight* lgt : lights) {
    delete lgt;
  }
  for (auto s : shadermap) {
    delete s.second;
  }
  for (auto s : shadows) {
    delete s;
  }
  delete configuration;
  delete this->eventBus;
}

GLSceneConfiguration* GLScene::getConfiguration() { return configuration; }
GLCamera& GLScene::getCamera() { return this->camera; }
Fragments& GLScene::getFragments() { return this->fragments; }
GLProjection& GLScene::getProjection() { return this->projection; }
void GLScene::setViewHeight(double h) {
  this->viewHeight = h;
  this->projection.height = h;
}
void GLScene::setViewWidth(double w) {
  this->viewWidth = w;
  this->projection.width = w;
}
void GLScene::setViewSize(double w, double h) {
  this->setViewWidth(w);
  this->setViewHeight(h);
}
double GLScene::getViewHeight() { return this->viewHeight; }
double GLScene::getViewWidth() { return this->viewWidth; }

GLShader* GLScene::getShader(IlluminationModel model) { return this->shadermap[model]; }

void GLScene::setAmbient(Color01 ambient) { this->ambient = ambient; }
Color01 GLScene::getAmbient() const { return this->ambient; }

void GLScene::addObj(GLObject* obj) {
  objs.push_back(obj);
  if (this->enabledShadow) {
    for (auto s : shadows) {
      s->refreshDepthMap();
    }
  }
}

void GLScene::addModel(GLModel* model) {
  models.push_back(model);
  if (this->enabledShadow) {
    for (auto s : shadows) {
      s->refreshDepthMap();
    }
  }
}

void GLScene::addLight(GLLight* lgt) {
  lgt->setEventBus(eventBus);
  lights.push_back(lgt);
  if (enabledShadow) {
    shadows.push_back(new GLShadowMapping(this, lgt));
    shadows.back()->refreshDepthMap();
  }
}
std::vector<GLLight*>& GLScene::getLights() { return this->lights; }  // 光源-阴影同步
std::vector<GLObject*>& GLScene::getObjs() { return this->objs; }     // 对象-阴影同步
std::vector<GLShadowMapping*>& GLScene::getShadows() { return this->shadows; }

Eigen::Matrix4d GLScene::viewportMatrix() {
  double hw = this->viewWidth / 2;
  double hh = this->viewHeight / 2;
  Eigen::Matrix4d viewMtx;
  viewMtx << hw, 0, 0, 0,  //
      0, hh, 0, 0,         //
      0, 0, 1, 0,          //
      hw, hh, 0, 1;        //
  return viewMtx;
}

void GLScene::calculateTransformMatrix() {
  // 视图变换矩阵 * 投影变换矩阵 * 视口变换矩阵
  transformMatrix = camera.viewMatrix() * projection.projMatrix() * viewportMatrix();
  // 逆变换矩阵 用于将屏幕坐标转换为世界坐标
  invTransformMatrix = transformMatrix.inverse();
}

// screen coordinator back to world coordinator
Vertice GLScene::screenVerticeBackToWorldVertice(double x, double y, double z, double w) {
  return screenVerticeBackToWorldVertice({x, y, z, w});
}

Vertice GLScene::screenVerticeBackToWorldVertice(Vertice v) {
  v = v.transpose() * invTransformMatrix;
  v /= v[3];
  return v;
}

Vertice GLScene::screenVerticeBackToCameraVertice(Vertice v) {
  v = v.transpose() * invTransformMatrix;
  v /= v[3];
  v = v.transpose() * this->getCamera().viewMatrix();
  return v;
}

Eigen::Matrix4d& GLScene::getInvTranformMatrix() { return this->invTransformMatrix; }

Fragments GLScene::initFragmentsBuffer() {
  Fragments fs(this->viewHeight, std::vector<Fragment>(this->viewWidth, Fragment::init()));
  return fs;
}

bool GLScene::isTriangleBack(Vertice& rp0, Vertice& rp1, Vertice& rp2, Normal& n0, Normal& n1,
                             Normal& n2) {
  Eigen::Vector3d p = ((rp0 + rp1 + rp2) / 3).head(3);
  Eigen::Vector3d n = (n0 + n1 + n2).head(3).normalized();
  Eigen::Vector3d c = this->getCamera().getPositionVertice().head(3);
  Eigen::Vector3d v1 = n;
  Eigen::Vector3d v2 = (c - p).head(3).normalized();
  return v1.dot(v2) < 0;
}

void GLScene::meshTransformToScreen(GLObject* obj) {
  assert(!obj->editing);

  // 计算变换矩阵(视图变换+投影变换+视口变换)及逆矩阵
  calculateTransformMatrix();

  // 视图变换 + 投影变换 + 视口变换
  obj->transformVerticesToScreen(this->transformMatrix);
}

// void GLScene::rasterize() {
//   fragments = initFragmentsBuffer();  // TODO clear rather than init new

//   for (GLObject* obj : objs) {
//     meshTransformToScreen(obj);
//     obj->rasterize(*this);
//   }
// }

void GLScene::rasterize() {
  GLTriangleShader shader;
  fragments = initFragmentsBuffer();  // TODO clear rather than init new
  calculateTransformMatrix();

  for (GLObject* obj : objs) {
    std::vector<GLPrimitive> primitives = obj->getPrimitives();
    for (GLPrimitive& primitive : primitives) {
      primitive.transformFromWorldToScreen(this->transformMatrix);
      std::vector<Triangle3> triangles = primitive.getTriangles();
      for (Triangle3& t : triangles) {
        shader.shade(t, primitive.getMaterial(), this->getCamera(), this->getLights(),
                     this->getShadows(), this->getAmbient(), this->getFragments(),
                     primitive.getInvMatrix());
      }
    }
  }

  for (GLModel* model : models) {
    std::vector<GLPrimitive> primitives = model->getPrimitives();
    for (GLPrimitive& primitive : primitives) {
      primitive.transformFromWorldToScreen(this->transformMatrix);
      std::vector<Triangle3> triangles = primitive.getTriangles();
      for (Triangle3& t : triangles) {
        shader.shade(t, primitive.getMaterial(), this->getCamera(), this->getLights(),
                     this->getShadows(), this->getAmbient(), this->getFragments(),
                     primitive.getInvMatrix());
      }
    }
  }
}

void GLScene::draw(QPainter& painter) {
  rasterize();

  for (int h = 0; h < this->viewHeight; ++h) {
    for (int w = 0; w < this->viewWidth; ++w) {
      Fragment fragment = fragments[h][w];
      if (fragment.depth < Fragment::DEPTH_INF) {  // clip
        QPen oldpen = painter.pen();

        painter.setPen(QPen(QColor(Color01Utils::red(fragment.color) * 255,
                                   Color01Utils::green(fragment.color) * 255,
                                   Color01Utils::blue(fragment.color) * 255),
                            1));
        painter.drawPoint(w, h);
        painter.setPen(oldpen);
      }
    }
  }
}

/*
GLSceneConfiguration
*/
void GLSceneConfiguration::setInterpolateMethod(InterpolateMethod method) {
  this->interpolateMethod = method;

  for (GLObject* obj : scene->getObjs()) {
    GLMesh* mesh = dynamic_cast<GLMesh*>(obj);
    if (mesh) {
      std::map<std::string, GLMaterial*>& materialmap = mesh->getMaterials();
      for (auto m : materialmap) {
        GLMaterial* material = m.second;
        InterpolateGLTexture* t =
            dynamic_cast<InterpolateGLTexture*>(material->getAmbientTexture());
        if (t) {
          t->setInterpolateMethod(method);
        }
        t = dynamic_cast<InterpolateGLTexture*>(material->getDiffuseTexture());
        if (t) {
          t->setInterpolateMethod(method);
        }
      }
    }
  }
}

InterpolateMethod GLSceneConfiguration::getInterpolateMethod() { return this->interpolateMethod; }

void GLSceneConfiguration::setTexCoordType(TexCoordType typ) {
  texCoordType = typ;
  GLTexture::texCoordType = typ;
}
TexCoordType GLSceneConfiguration::getTexCoordType() { return texCoordType; }
}  // namespace qtgl