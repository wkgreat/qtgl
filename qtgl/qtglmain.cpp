#include <QApplication>
#include "gltfmodel.hpp"
#include "mesh.hpp"
#include "model.hpp"
#include "render.hpp"

int main(int argc, char* argv[]) {
  QApplication app(argc, argv);
  QWidget* window = new QWidget;
  window->setWindowTitle("QTGL");
  window->setGeometry(0, 0, 1000, 1000);
  QGridLayout* layout = new QGridLayout;

  qtgl::GLRenderWidget widget;
  widget.setFixedSize(1000, 1000);

  // mesh axis
  // std::string axisobjpath = "E:\\codes\\practice\\qt-learning\\data\\xyz_axis\\xyz_axis.obj";
  // qtgl::GLMesh* axisMesh = qtgl::GLMesh::readFromObjFile(axisobjpath);
  // axisMesh->startEditing();
  // axisMesh->setModelMatrix(qtgl::AffineUtils::scaleMtx(2, 2, 2));
  // axisMesh->finishEditing();
  // widget.getScene().addObj(axisMesh);
  // mesh f16
  // std::string f16objpath =
  //     "E:\\codes\\practice\\qt-learning\\data\\F16_Fighting_Falcon\\F16fin.obj";
  // qtgl::GLMesh* f16Mesh = qtgl::GLMesh::readFromObjFile(f16objpath);
  // // widget.getScene().addObj(f16Mesh);
  // widget.getScene().addModel(f16Mesh);

  std::string helmetpath = "E:\\codes\\practice\\qtgl\\data\\DamagedHelmet\\DamagedHelmet.gltf";
  qtgl::GLTFModel* model = new qtgl::GLTFModel(helmetpath);
  widget.getScene().addModel(model);

  // camera an projection
  widget.getScene().getCamera().lookAt(-1, 1, 1, 0, 0, 0);
  widget.getScene().getProjection().mode = qtgl::GLProjectionMode::PRESPECTIVE;

  // light
  qtgl::PointGLLight* lgt = new qtgl::PointGLLight;
  lgt->setIntensity({1, 1, 1, 1});
  lgt->setPosition({-500, 500, 500, 0});
  widget.getScene().addLight(lgt);
  widget.getScene().setAmbient({0.5, 0.5, 0.5, 0.5});
  widget.getScene().disableShadow();
  layout->addWidget(&widget, 0, 0);

  // helper
  qtgl::SceneHelper helper;
  helper.setScene(&(widget.getScene()));
  layout->addWidget(&helper, 1, 0);

  qtgl::TextureHelper texHelper;
  texHelper.setScene(&(widget.getScene()));
  layout->addWidget(&texHelper, 2, 0);

  qtgl::GLPointLightHelper lgthelper;
  lgthelper.setLight(lgt);
  layout->addWidget(&lgthelper, 3, 0);

  window->setLayout(layout);
  window->show();

  return app.exec();
}