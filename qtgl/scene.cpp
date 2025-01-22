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
  delete configuration;
}

void GLScene::meshTransformToScreen(GLObject* obj) {
  // // 渲染 TODO 渲染移动至光栅化期间
  // Vertice cameraPos{camera.getPosX(), camera.getPosY(), camera.getPosZ(), 0};
  // obj->shadeVertices(shader, lights, cameraPos);

  // 计算变换矩阵(视图变换+投影变换+视口变换)及逆矩阵
  calculateTransformMatrix();

  // 准备变换
  obj->prepareTransform();
  // 模型变换
  obj->transformWithModelMatrix();
  // 视图变换 + 投影变换 + 视口变换
  obj->transformVerticesWithMatrix(this->transformMatrix);
}

void GLScene::draw(QPainter& painter) {
  fragments = initFragmentsBuffer();  // TODO clear rather than init new

  for (GLObject* obj : objs) {
    meshTransformToScreen(obj);
    obj->rasterize(*this);
  }
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