#pragma once

#include <algorithm>
#include <cmath>
#include "define.hpp"
#include "material.hpp"

namespace qtgl {

struct GLLight {
  Color01 intensity;
  virtual Eigen::Vector3d uvLight(Vertice& pos) = 0;  // l unit vector
};

struct DirectionalGLLight : public GLLight {
  Eigen::Vector3d d;  // direction
  Eigen::Vector3d uvLight(Vertice& pos) { return (d * -1).normalized(); }
};

struct PointGLLight : public GLLight {
  Vertice position;
  Eigen::Vector3d uvLight(Vertice& pos) { return (position - pos).head(3).normalized(); }
};

struct GLShader {
  virtual Color01 shade(std::vector<GLLight*>& lights, Color01 ambient, GLMaterial* material,
                        Vertice& position, Eigen::Vector3d& uvNormal, Eigen::Vector3d& uvView,
                        TexCoord* coord) = 0;
};

struct LambertianGLShader : public GLShader {
  Color01 shade(std::vector<GLLight*>& lights, Color01 ambient, GLMaterial* material,
                Vertice& position, Eigen::Vector3d& uvNormal, Eigen::Vector3d& uvView,
                TexCoord* coord) {
    Eigen::Vector3d uvLight;
    Color01 r(0, 0, 0, 0);
    r = r + ambient.cwiseProduct(material->getAmbient(coord));
    Color01 p(0, 0, 0, 0);
    for (GLLight* light : lights) {
      uvLight = light->uvLight(position);
      p = p + (std::max(0.0, uvLight.dot(uvNormal)))*light->intensity;
    }
    r = r + p.cwiseProduct(material->getDiffuse(coord));
    return r;
  }
};
struct LambertialBlinnPhongGLShader : public GLShader {
  Color01 shade(std::vector<GLLight*>& lights, Color01 ambient, GLMaterial* material,
                Vertice& position, Eigen::Vector3d& uvNormal, Eigen::Vector3d& uvView,
                TexCoord* coord) {
    Eigen::Vector3d uvLight;
    Eigen::Vector3d uvHalf;
    Color01 r(0, 0, 0, 0);
    r = r + ambient.cwiseProduct(material->getAmbient(coord));
    Color01 d(0, 0, 0, 0);
    Color01 s(0, 0, 0, 0);
    for (GLLight* light : lights) {
      uvLight = light->uvLight(position);
      uvHalf = (uvView + uvLight).normalized();
      d = d + (std::max(0.0, uvLight.dot(uvNormal)))*light->intensity;
      s = s + std::pow(std::max(0.0, uvHalf.dot(uvNormal)), material->getSpecularHighlight()) *
                  light->intensity;
    }
    r = r + d.cwiseProduct(material->getDiffuse(coord));
    r = r + s.cwiseProduct(material->getSpecular());
    r = Color01Utils::clamp(r);
    return r;
  }
};

}  // namespace qtgl