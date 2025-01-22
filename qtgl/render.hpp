#pragma once

#include <QComboBox>
#include <QGridLayout>
#include <QLabel>
#include <QMouseEvent>
#include <QSlider>
#include <QTimer>
#include <QWheelEvent>
#include <QWidget>
#include <functional>
#include "scene.hpp"

namespace qtgl {

class GLRenderWidget : public QWidget {
 private:
  GLScene scene;
  std::function<void(GLScene&)> beforeRender = [](GLScene& scene) {};

  double wheelMoveForwardFactor = 1.0 / 120 * 20;
  int mouseLastX, mouseLastY;
  double mouseXRoundFactor = 0.01;
  double mouseYRoundFactor = -0.01;
  double mouseXMoveFactor = 20;
  double mouseYMoveFactor = 20;

 public:
  GLRenderWidget(QWidget* parent = nullptr) : QWidget(parent) {
    QTimer* timer = new QTimer(this);                                  // 创建定时器
    timer->setInterval(20);                                            // 定时间隔单位ms
    connect(timer, &QTimer::timeout, this, &GLRenderWidget::refresh);  // 定时器关联refresh
    timer->start();                                                    // 定时启动
  }

  void setFixedSize(int w, int h) {
    scene.setViewWidth(w);
    scene.setViewHeight(h);
    QWidget::setFixedSize(w, h);
  }

  void setBeforeRender(std::function<void(GLScene&)> beforeRender) {
    this->beforeRender = beforeRender;
  }

  GLScene& getScene() { return scene; }

  void refresh() {
    beforeRender(scene);
    this->update();
  }

  void paintEvent(QPaintEvent* event) override {
    QPainter painter(this);
    painter.eraseRect(0, 0, this->width(), this->height());  // 清除画布
    scene.draw(painter);
  }

  void mousePressEvent(QMouseEvent* event) override {
    QPoint pos = event->pos();
    mouseLastX = pos.x();
    mouseLastY = pos.y();
  }
  void mouseMoveEvent(QMouseEvent* event) override {
    // TODO move follow mouse
    QPoint pos = event->pos();
    if (event->buttons() & Qt::MiddleButton) {  // 平移;
      Vertice lastMousePos(mouseLastX, mouseLastY, 1, 1);
      Vertice mousePos(pos.x(), pos.y(), 1, 1);
      lastMousePos = this->scene.screenVerticeBackToCameraVertice(lastMousePos);
      mousePos = this->scene.screenVerticeBackToCameraVertice(mousePos);
      this->scene.getCamera().move((mousePos[0] - lastMousePos[0]) * mouseXMoveFactor,
                                   (mousePos[1] - lastMousePos[1]) * mouseXMoveFactor);
    } else if (event->buttons() & Qt::LeftButton) {  // 环绕
      int dx = pos.x() - mouseLastX;
      int dy = pos.y() - mouseLastY;
      this->scene.getCamera().round(dx * mouseXRoundFactor, dy * mouseYRoundFactor);
    } else if (event->buttons() & Qt::RightButton) {
      // do nothing
    }
    mouseLastX = pos.x();
    mouseLastY = pos.y();
  }
  void wheelEvent(QWheelEvent* event) override {  // 缩放
    this->scene.getCamera().zoom(event->angleDelta().y() * wheelMoveForwardFactor);
  }
};

class GLPointLightHelper : public QWidget {
 public:
  PointGLLight* lgt;
  QLabel label1;
  QLabel label2;
  QLabel label3;

  QSlider slider1;
  QSlider slider2;
  QSlider slider3;

  QGridLayout layout;
  GLPointLightHelper(QWidget* parent = nullptr) : QWidget(parent) {}
  void setLight(PointGLLight* lgt) {
    this->lgt = lgt;
    label1.setText(QString("Light PosX: ") + QString::number(this->lgt->position[0]));
    label2.setText(QString("Light PosY: ") + QString::number(this->lgt->position[1]));
    label3.setText(QString("Light PosZ: ") + QString::number(this->lgt->position[2]));

    slider1.setOrientation(Qt::Horizontal);
    slider1.setMinimum(-600);
    slider1.setMaximum(600);
    slider1.setSingleStep(1);
    slider1.setValue(this->lgt->position[0]);
    slider1.setTracking(true);

    connect(&slider1, &QSlider::valueChanged, [&](int value) {
      this->lgt->position[0] = value;
      label1.setText(QString("Light PosX: ") + QString::number(this->lgt->position[0]));
    });

    slider2.setOrientation(Qt::Horizontal);
    slider2.setMinimum(-600);
    slider2.setMaximum(600);
    slider2.setSingleStep(1);
    slider2.setValue(this->lgt->position[1]);
    slider2.setTracking(true);

    connect(&slider2, &QSlider::valueChanged, [&](int value) {
      this->lgt->position[1] = value;
      label2.setText(QString("Light PosY: ") + QString::number(this->lgt->position[1]));
    });

    slider3.setOrientation(Qt::Horizontal);
    slider3.setMinimum(-600);
    slider3.setMaximum(600);
    slider3.setSingleStep(1);
    slider3.setValue(this->lgt->position[2]);
    slider3.setTracking(true);

    connect(&slider3, &QSlider::valueChanged, [&](int value) {
      this->lgt->position[2] = value;
      label3.setText(QString("Light PosZ: ") + QString::number(this->lgt->position[2]));
    });

    layout.addWidget(&label1, 0, 0);
    layout.addWidget(&slider1, 0, 1);
    layout.addWidget(&label2, 1, 0);
    layout.addWidget(&slider2, 1, 1);
    layout.addWidget(&label3, 2, 0);
    layout.addWidget(&slider3, 2, 1);

    this->setLayout(&layout);
  }
  ~GLPointLightHelper() = default;
};

class TextureHelper : public QWidget {
 public:
  GLScene* scene;
  QLabel labelInterpolateMethod;
  QComboBox boxInterpolateMethod;
  QLabel labelTexcoordMethod;
  QComboBox boxTexcoordMethod;
  QGridLayout layout;

  TextureHelper(QWidget* parent = nullptr) : QWidget(parent) {}
  ~TextureHelper() = default;

  void setScene(GLScene* scene) {
    this->scene = scene;
    labelInterpolateMethod.setText(QString("Texture Interpolation Method: "));
    boxInterpolateMethod.addItem(QString("bilinear"));
    boxInterpolateMethod.addItem(QString("linear"));
    connect(&boxInterpolateMethod, &QComboBox::currentTextChanged, [&](QString text) {
      if (text == QString("linear")) {
        this->scene->getConfiguration()->setInterpolateMethod(InterpolateMethod::LIINEAR);
      } else if (text == QString("bilinear")) {
        this->scene->getConfiguration()->setInterpolateMethod(InterpolateMethod::BILINEAR);
      }
    });
    layout.addWidget(&labelInterpolateMethod, 1, 0);
    layout.addWidget(&boxInterpolateMethod, 1, 1);

    labelTexcoordMethod.setText(QString("Texcoord Type: "));
    boxTexcoordMethod.addItem(QString("perspective correct texcoord"));
    boxTexcoordMethod.addItem(QString("basic texcoord"));
    connect(&boxTexcoordMethod, &QComboBox::currentTextChanged, [&](QString text) {
      if (text == QString("basic texcoord")) {
        this->scene->getConfiguration()->setTexCoordType(TexCoordType::BASIC);
      } else if (text == QString("perspective correct texcoord")) {
        this->scene->getConfiguration()->setTexCoordType(TexCoordType::PERSPECTIVE_CORRECT);
      }
    });
    layout.addWidget(&labelTexcoordMethod, 1, 2);
    layout.addWidget(&boxTexcoordMethod, 1, 3);

    this->setLayout(&layout);
  }
};

class SceneHelper : public QWidget {
 public:
  GLScene* scene;
  QLabel label1;
  QLabel label2;
  QLabel label3;
  QSlider slider1;
  QSlider slider2;
  QSlider slider3;

  QLabel label4;
  QLabel label5;
  QLabel label6;
  QSlider slider4;
  QSlider slider5;
  QSlider slider6;

  QGridLayout layout;
  SceneHelper(QWidget* parent = nullptr) : QWidget(parent) {}
  void setScene(GLScene* scene) {
    this->scene = scene;

    label1.setText(QString("Heading: ") +
                   QString::number(MathUtils::toDegree(this->scene->getCamera().getHeading())));
    label2.setText(QString("Pitch: ") +
                   QString::number(MathUtils::toDegree(this->scene->getCamera().getPitch())));
    label3.setText(QString("Roll: ") +
                   QString::number(MathUtils::toDegree(this->scene->getCamera().getRool())));
    label4.setText(QString("PosX: ") + QString::number(this->scene->getCamera().getPosX()));
    label5.setText(QString("PosY: ") + QString::number(this->scene->getCamera().getPosY()));
    label6.setText(QString("PosZ: ") + QString::number(this->scene->getCamera().getPosZ()));

    slider1.setOrientation(Qt::Horizontal);
    slider1.setMinimum(-360);
    slider1.setMaximum(360);
    slider1.setSingleStep(1);
    slider1.setValue(MathUtils::toDegree(this->scene->getCamera().getHeading()));
    slider1.setTracking(true);
    connect(&slider1, &QSlider::valueChanged, [&](int value) {
      this->scene->getCamera().setHeading(MathUtils::toRadians(value));
      label1.setText(QString("Heading: ") +
                     QString::number(MathUtils::toDegree(this->scene->getCamera().getHeading())));
    });

    slider2.setOrientation(Qt::Horizontal);
    slider2.setMinimum(-360);
    slider2.setMaximum(360);
    slider2.setSingleStep(1);
    slider2.setValue(MathUtils::toDegree(this->scene->getCamera().getPitch()));
    slider2.setTracking(true);
    connect(&slider2, &QSlider::valueChanged, [&](int value) {
      this->scene->getCamera().setPitch(MathUtils::toRadians(value));
      label2.setText(QString("Pitch: ") +
                     QString::number(MathUtils::toDegree(this->scene->getCamera().getPitch())));
    });

    slider3.setOrientation(Qt::Horizontal);
    slider3.setMinimum(-360);
    slider3.setMaximum(360);
    slider3.setSingleStep(1);
    slider3.setValue(MathUtils::toDegree(this->scene->getCamera().getRool()));
    slider3.setTracking(true);

    connect(&slider3, &QSlider::valueChanged, [&](int value) {
      this->scene->getCamera().setRoll(MathUtils::toRadians(value));
      label3.setText(QString("Roll: ") +
                     QString::number(MathUtils::toDegree(this->scene->getCamera().getRool())));
    });

    slider4.setOrientation(Qt::Horizontal);
    slider4.setMinimum(-600);
    slider4.setMaximum(600);
    slider4.setSingleStep(1);
    slider4.setValue(this->scene->getCamera().getPosX());
    slider4.setTracking(true);

    connect(&slider4, &QSlider::valueChanged, [&](int value) {
      this->scene->getCamera().setPosX(value);
      label4.setText(QString("PosX: ") + QString::number(this->scene->getCamera().getPosX()));
    });

    slider5.setOrientation(Qt::Horizontal);
    slider5.setMinimum(-600);
    slider5.setMaximum(600);
    slider5.setSingleStep(1);
    slider5.setValue(this->scene->getCamera().getPosY());
    slider5.setTracking(true);

    connect(&slider5, &QSlider::valueChanged, [&](int value) {
      this->scene->getCamera().setPosY(value);
      label5.setText(QString("PosY: ") + QString::number(this->scene->getCamera().getPosY()));
    });

    slider6.setOrientation(Qt::Horizontal);
    slider6.setMinimum(-600);
    slider6.setMaximum(600);
    slider6.setSingleStep(1);
    slider6.setValue(this->scene->getCamera().getPosZ());
    slider6.setTracking(true);

    connect(&slider6, &QSlider::valueChanged, [&](int value) {
      this->scene->getCamera().setPosZ(value);
      label6.setText(QString("PosZ: ") + QString::number(this->scene->getCamera().getPosZ()));
    });

    layout.addWidget(&label1, 0, 0);
    layout.addWidget(&slider1, 0, 1);
    layout.addWidget(&label2, 1, 0);
    layout.addWidget(&slider2, 1, 1);
    layout.addWidget(&label3, 2, 0);
    layout.addWidget(&slider3, 2, 1);

    layout.addWidget(&label4, 0, 2);
    layout.addWidget(&slider4, 0, 3);
    layout.addWidget(&label5, 1, 2);
    layout.addWidget(&slider5, 1, 3);
    layout.addWidget(&label6, 2, 2);
    layout.addWidget(&slider6, 2, 3);

    this->setLayout(&layout);
  }
  ~SceneHelper() = default;
};

}  // namespace qtgl