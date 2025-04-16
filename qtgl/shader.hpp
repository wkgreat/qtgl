#pragma once

#include <algorithm>
#include <cmath>
#include "define.hpp"
#include "event.hpp"
#include "material.hpp"

namespace qtgl {

class GLLight {
 private:
  Color01 intensity;
  GLEventBus* eventBus = nullptr;

 public:
  virtual Eigen::Vector3d uvLight(Vertice& pos) = 0;  // l unit vector
  Color01 getIntensity() { return intensity; }
  void setIntensity(Color01 c) { this->intensity = c; }
  void setEventBus(GLEventBus* bus) { this->eventBus = bus; }
  GLEventBus* getEventBus() { return this->eventBus; }
};

class DirectionalGLLight : public GLLight {
 private:
  Eigen::Vector3d d;  // direction
 public:
  Eigen::Vector3d uvLight(Vertice& pos) { return (d * -1).normalized(); }
  Eigen::Vector3d getDirection() { return d; }
};

class PointGLLight : public GLLight {
 private:
  Vertice position;

 public:
  Eigen::Vector3d uvLight(Vertice& pos) { return (position - pos).head(3).normalized(); }
  Vertice getPosition() { return position; }
  void setPosition(Vertice pos) {
    position = pos;
    if (this->getEventBus() != nullptr) {
      this->getEventBus()->fire(GLEvent::CHANGE_LIGHT, this);
    }
  }
  void setPosition(int idx, double v) {
    this->position[idx] = v;
    if (this->getEventBus() != nullptr) {
      this->getEventBus()->fire(GLEvent::CHANGE_LIGHT, this);
    }
  }
};

struct GLShader {
  virtual Color01 shade(std::vector<GLLight*>& lights, std::vector<int>& lightInShadow,
                        Color01 ambient, GLMaterial* material, Vertice& position,
                        Eigen::Vector3d& uvNormal, Eigen::Vector3d& uvView, TexCoord* coord) = 0;
  virtual Color01 shade2(Triangle2& triangle, Vertice& worldPos, GLCamera& camera,
                         Triangle2::BarycentricCoordnates& centricCoord,
                         std::vector<GLLight*>& lights, std::vector<int>& isLightShadow,
                         GLMaterial& material, Color01 ambient) = 0;
};

struct LambertianGLShader : public GLShader {
  Color01 shade(std::vector<GLLight*>& lights, std::vector<int>& lightInShadow, Color01 ambient,
                GLMaterial* material, Vertice& position, Eigen::Vector3d& uvNormal,
                Eigen::Vector3d& uvView, TexCoord* coord) {
    Eigen::Vector3d uvLight;
    Color01 r(0, 0, 0, 0);
    r = r + ambient.cwiseProduct(material->getAmbient(coord));
    Color01 p(0, 0, 0, 0);
    for (int i = 0; i < lights.size(); ++i) {
      if (!lightInShadow[i]) {
        uvLight = lights[i]->uvLight(position);
        p = p + (std::max(0.0, uvLight.dot(uvNormal))) * lights[i]->getIntensity();
      }
    }
    r = r + p.cwiseProduct(material->getDiffuse(coord));
    return r;
  }

  Color01 shade2(Triangle2& triangle, Vertice& worldPos, GLCamera& camera,
                 Triangle2::BarycentricCoordnates& centricCoord, std::vector<GLLight*>& lights,
                 std::vector<int>& isLightShadow, GLMaterial& material, Color01 ambient) {
    Color01 color;
    Normal uvNormal =
        (centricCoord.alpha * triangle.getNormal0() + centricCoord.beta * triangle.getNormal1() +
         centricCoord.gamma * triangle.getNormal2())
            .normalized();
    Normal uvView = (camera.getPositionVertice().head(3) - worldPos.head(3)).normalized();
    TexCoord txtcoord = GLTexture::interpolateTexCoord(triangle, centricCoord.alpha,
                                                       centricCoord.beta, centricCoord.gamma);

    color = this->shade(lights, isLightShadow, ambient, &material, worldPos, uvNormal, uvView,
                        &txtcoord);
    return color;
  }
};

struct LambertialBlinnPhongGLShader : public GLShader {
  Color01 shade(std::vector<GLLight*>& lights, std::vector<int>& lightInShadow, Color01 ambient,
                GLMaterial* material, Vertice& position, Eigen::Vector3d& uvNormal,
                Eigen::Vector3d& uvView, TexCoord* coord) {
    Eigen::Vector3d uvLight;
    Eigen::Vector3d uvHalf;
    Color01 r(0, 0, 0, 0);
    r = r + ambient.cwiseProduct(material->getAmbient(coord));
    Color01 d(0, 0, 0, 0);
    Color01 s(0, 0, 0, 0);
    for (int i = 0; i < lights.size(); ++i) {
      if (!lightInShadow[i]) {
        uvLight = lights[i]->uvLight(position);
        uvHalf = (uvView + uvLight).normalized();
        d = d + (std::max(0.0, uvLight.dot(uvNormal))) * lights[i]->getIntensity();
        s = s + std::pow(std::max(0.0, uvHalf.dot(uvNormal)), material->getSpecularHighlight()) *
                    lights[i]->getIntensity();
      }
    }
    r = r + d.cwiseProduct(material->getDiffuse(coord));
    r = r + s.cwiseProduct(material->getSpecular());
    r = Color01Utils::clamp(r);
    return r;
  }

  Color01 shade2(Triangle2& triangle, Vertice& worldPos, GLCamera& camera,
                 Triangle2::BarycentricCoordnates& centricCoord, std::vector<GLLight*>& lights,
                 std::vector<int>& isLightShadow, GLMaterial& material, Color01 ambient) {
    Color01 color;
    Normal uvNormal =
        (centricCoord.alpha * triangle.getNormal0() + centricCoord.beta * triangle.getNormal1() +
         centricCoord.gamma * triangle.getNormal2())
            .normalized();
    Normal uvView = (camera.getPositionVertice().head(3) - worldPos.head(3)).normalized();
    TexCoord txtcoord = GLTexture::interpolateTexCoord(triangle, centricCoord.alpha,
                                                       centricCoord.beta, centricCoord.gamma);

    color = this->shade(lights, isLightShadow, ambient, &material, worldPos, uvNormal, uvView,
                        &txtcoord);
    return color;
  }
};

}  // namespace qtgl