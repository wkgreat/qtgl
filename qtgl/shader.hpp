#pragma once

#include <algorithm>
#include <cmath>
#include "camera.hpp"
#include "define.hpp"
#include "event.hpp"
#include "gltfmodel.hpp"
#include "material.hpp"
#include "primitive.hpp"

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

struct GLShaderBase {};

struct GLShader : public GLShaderBase {
  virtual Color01 shade(std::vector<GLLight*>& lights, std::vector<int>& lightInShadow,
                        Color01 ambient, GLMaterialBase* material, Vertice& position,
                        Eigen::Vector3d& uvNormal, Eigen::Vector3d& uvView, TexCoord* coord) = 0;
  virtual Color01 shade2(Triangle3& triangle, Vertice& worldPos, GLCamera& camera,
                         Triangle3::BarycentricCoordnates& centricCoord,
                         std::vector<GLLight*>& lights, std::vector<int>& isLightShadow,
                         GLMaterialBase* material, Color01 ambient) = 0;
};

struct LambertianGLShader : public GLShader {
  Color01 shade(std::vector<GLLight*>& lights, std::vector<int>& lightInShadow, Color01 ambient,
                GLMaterialBase* material, Vertice& position, Eigen::Vector3d& uvNormal,
                Eigen::Vector3d& uvView, TexCoord* coord) {
    GLMaterial* m = reinterpret_cast<GLMaterial*>(material);
    Eigen::Vector3d uvLight;
    Color01 r(0, 0, 0, 0);
    r = r + ambient.cwiseProduct(m->getAmbient(coord));
    Color01 p(0, 0, 0, 0);
    for (int i = 0; i < lights.size(); ++i) {
      if (!lightInShadow[i]) {
        uvLight = lights[i]->uvLight(position);
        p = p + (std::max(0.0, uvLight.dot(uvNormal))) * lights[i]->getIntensity();
      }
    }
    r = r + p.cwiseProduct(m->getDiffuse(coord));
    return r;
  }

  Color01 shade2(Triangle3& triangle, Vertice& worldPos, GLCamera& camera,
                 Triangle3::BarycentricCoordnates& centricCoord, std::vector<GLLight*>& lights,
                 std::vector<int>& isLightShadow, GLMaterialBase* material, Color01 ambient) {
    Color01 color;
    Normal uvNormal =
        (centricCoord.alpha * triangle.getNormal0() + centricCoord.beta * triangle.getNormal1() +
         centricCoord.gamma * triangle.getNormal2())
            .normalized();
    Normal uvView = (camera.getPositionVertice().head(3) - worldPos.head(3)).normalized();

    TexCoord txtcoord;
    if (triangle.hasTexCoords(0)) {
      txtcoord = GLTexture::interpolateTexCoord(triangle, 0, centricCoord.alpha, centricCoord.beta,
                                                centricCoord.gamma);
    } else {
      txtcoord = {0, 0};
    }

    color = this->shade(lights, isLightShadow, ambient, material, worldPos, uvNormal, uvView,
                        &txtcoord);
    return color;
  }
};

struct LambertialBlinnPhongGLShader : public GLShader {
  Color01 shade(std::vector<GLLight*>& lights, std::vector<int>& lightInShadow, Color01 ambient,
                GLMaterialBase* material, Vertice& position, Eigen::Vector3d& uvNormal,
                Eigen::Vector3d& uvView, TexCoord* coord) {
    GLMaterial* m = reinterpret_cast<GLMaterial*>(material);
    Eigen::Vector3d uvLight;
    Eigen::Vector3d uvHalf;
    Color01 r(0, 0, 0, 0);
    r = r + ambient.cwiseProduct(m->getAmbient(coord));
    Color01 d(0, 0, 0, 0);
    Color01 s(0, 0, 0, 0);
    for (int i = 0; i < lights.size(); ++i) {
      if (!lightInShadow[i]) {
        uvLight = lights[i]->uvLight(position);
        uvHalf = (uvView + uvLight).normalized();
        d = d + (std::max(0.0, uvLight.dot(uvNormal))) * lights[i]->getIntensity();
        s = s + std::pow(std::max(0.0, uvHalf.dot(uvNormal)), m->getSpecularHighlight()) *
                    lights[i]->getIntensity();
      }
    }
    r = r + d.cwiseProduct(m->getDiffuse(coord));
    r = r + s.cwiseProduct(m->getSpecular());
    r = Color01Utils::clamp(r);
    return r;
  }

  Color01 shade2(Triangle3& triangle, Vertice& worldPos, GLCamera& camera,
                 Triangle3::BarycentricCoordnates& centricCoord, std::vector<GLLight*>& lights,
                 std::vector<int>& isLightShadow, GLMaterialBase* material, Color01 ambient) {
    Color01 color;
    Normal uvNormal =
        (centricCoord.alpha * triangle.getNormal0() + centricCoord.beta * triangle.getNormal1() +
         centricCoord.gamma * triangle.getNormal2())
            .normalized();
    Normal uvView = (camera.getPositionVertice().head(3) - worldPos.head(3)).normalized();
    TexCoord txtcoord;
    if (triangle.hasTexCoords(0)) {
      txtcoord = GLTexture::interpolateTexCoord(triangle, 0, centricCoord.alpha, centricCoord.beta,
                                                centricCoord.gamma);
    } else {
      txtcoord = {0, 0};
    }

    color = this->shade(lights, isLightShadow, ambient, material, worldPos, uvNormal, uvView,
                        &txtcoord);
    return color;
  }
};

struct PBRGLShader : public GLShaderBase {
  Color01 shade(std::vector<GLLight*>& lights, std::vector<int>& lightInShadow, Color01 ambient,
                GLMaterialBase* material, Vertice& position, Eigen::Vector3d& uvNormal,
                Eigen::Vector3d& uvView, TexCoord* coord) {
    return {1, 0, 0, 1};
  }

  inline double chi(double v) { return v > 0 ? 1 : 0; }
  inline double d_func(double alpha, Eigen::Vector3d N, Eigen::Vector3d H) {
    double a = pow(alpha, 2.0) * chi(N.dot(H));
    double b = MathUtils::PI * pow(pow(N.dot(H), 2.0) * (pow(alpha, 2.0) - 1) + 1, 2.0);
    return a / b;
  }
  inline double g1_func(Eigen::Vector3d N, Eigen::Vector3d H, Eigen::Vector3d X, double alpha) {
    double a = 2 * abs(N.dot(X)) * chi(H.dot(X));
    double b = abs(N.dot(X)) + sqrt(pow(alpha, 2.0) + (1 - pow(alpha, 2.0)) * pow(N.dot(X), 2.0));
    return a / b;
  }

  inline double g_func(Eigen::Vector3d N, Eigen::Vector3d H, Eigen::Vector3d L, Eigen::Vector3d V,
                       double alpha) {
    return g1_func(N, H, L, alpha) * g1_func(N, H, V, alpha);
  }

  Color01 brdf(Color01 baseColor, double metallic, double roughness, Eigen::Vector3d N,
               Eigen::Vector3d L, Eigen::Vector3d V, Eigen::Vector3d H) {
    Color01 black = {0, 0, 0, 1};
    Color01 one = {1, 1, 1, 1};
    Color01 c004 = {0.04, 0.04, 0.04, 1};
    Color01 c_diff = Color01Utils::mix(baseColor, black, metallic);
    Color01 f0 = Color01Utils::mix(c004, baseColor, metallic);
    double alpha = pow(roughness, 2.0);
    Color01 F = f0 + (one - f0) * pow(1 - abs(V.dot(H)), 5.0);
    Color01 f_diffuse = ((one - F) * (1 / MathUtils::PI)).cwiseProduct(c_diff);
    Color01 f_specular =
        F * d_func(alpha, N, H) * g_func(N, H, L, V, alpha) / (4 * abs(V.dot(N)) * abs(L.dot(N)));
    return f_diffuse + f_specular;
  }
  Color01 addOcclusion(Color01 color, double occlusion, double strength) {
    color = color * (1.0 + strength * (occlusion - 1.0));
    return color;
  }
  Color01 addEmmissive(Color01 color, Eigen::Vector3d emmissive, Eigen::Vector3d factor) {
    emmissive = emmissive.cwiseProduct(factor);
    color.head(3) += emmissive;
    return color;
  }

  Color01 shade2(Triangle3& triangle, Vertice& worldPos, GLCamera& camera,
                 Triangle3::BarycentricCoordnates& centricCoord, std::vector<GLLight*>& lights,
                 std::vector<int>& isLightShadow, GLMaterialBase* material, Color01 ambient) {
    GLTFMaterial* m = reinterpret_cast<GLTFMaterial*>(material);

    Color01 finalcolor = {0, 0, 0, 1};
    Normal N =
        (centricCoord.alpha * triangle.getNormal0() + centricCoord.beta * triangle.getNormal1() +
         centricCoord.gamma * triangle.getNormal2())
            .normalized();  // TODO 使用法线贴图
    Eigen::Vector3d V = (camera.getPositionVertice().head(3) - worldPos.head(3)).normalized();

    // base color
    Color01 basecolor = {1, 0, 0, 1};
    int k = m->getBaseColorTexCoordN();
    TexCoord t = {0, 0};
    if (k != -1) {
      t = GLTexture::interpolateTexCoord(triangle, k, centricCoord.alpha, centricCoord.beta,
                                         centricCoord.gamma);
    }
    basecolor = m->getBaseColor(t);

    // metallic roughness

    t = {0, 0};
    k = m->getMetallocRougnnessTexCoordN();
    if (k != -1) {
      t = GLTexture::interpolateTexCoord(triangle, k, centricCoord.alpha, centricCoord.beta,
                                         centricCoord.gamma);
    }
    Eigen::Vector2d mr = m->getMetallicRoughness(t);
    double metallic = mr[0];
    double roughness = mr[1];

    // brdf
    for (GLLight* lgt : lights) {
      // TODO check is in shadow;
      Eigen::Vector3d L = lgt->uvLight(worldPos);
      Eigen::Vector3d H = (L + V).normalized();
      Color01 c = brdf(basecolor, metallic, roughness, N, L, V, H);
      // Color01 c = basecolor;
      finalcolor = finalcolor + c;
    }

    // acclusion
    double acclusion = 1.0;         // TODO get acclusion;
    double acclusionStrengh = 1.0;  // TODO get acclusion strength;
    finalcolor = addOcclusion(finalcolor, acclusion, acclusionStrengh);

    // emmissive
    t = {0, 0};
    k = m->getEmissiveTexCoordN();
    if (k != -1) {
      t = GLTexture::interpolateTexCoord(triangle, k, centricCoord.alpha, centricCoord.beta,
                                         centricCoord.gamma);
    }
    Eigen::Vector3d emissive = m->getEmissive(t).head(3);
    Eigen::Vector3d emissiveFactor(m->getEmissiveFactor());
    finalcolor = addEmmissive(finalcolor, emissive, emissiveFactor);

    finalcolor = Color01Utils::clamp(finalcolor);
    finalcolor[3] = 1;

    return finalcolor;
  };
};

class GLShadowMapping;

struct GLTriangleShader {
  static bool isTriangleBack(GLCamera& camera, Triangle3& triangle) {
    Vertice wp0 = triangle.getWorldPos0();
    Vertice wp1 = triangle.getWorldPos0();
    Vertice wp2 = triangle.getWorldPos0();
    Normal n0 = triangle.getNormal0();
    Normal n1 = triangle.getNormal1();
    Normal n2 = triangle.getNormal2();

    Eigen::Vector3d p = ((wp0 + wp1 + wp2) / 3).head(3);
    Eigen::Vector3d n = (n0 + n1 + n2).head(3).normalized();
    Eigen::Vector3d c = camera.getPositionVertice().head(3);
    Eigen::Vector3d v1 = n;
    Eigen::Vector3d v2 = (c - p).head(3).normalized();
    return v1.dot(v2) < 0;
  };
  static Vertice verticeTransform(Eigen::Matrix4d& m, Vertice v) {
    v = v.transpose() * m;
    v /= v[3];
    return v;
  }
  static GLShaderBase* getShader(IlluminationModel illumination) {
    if (illumination == IlluminationModel::RANDOM) {
      return nullptr;
    } else if (illumination == IlluminationModel::CONSTANT) {
      // TODO
      return nullptr;
    } else if (illumination == IlluminationModel::LAMBERTIAN) {
      return new LambertianGLShader();
    } else if (illumination == IlluminationModel::LAMBERTIAN_BLINN_PHONG) {
      return new LambertialBlinnPhongGLShader();
    } else if (illumination == IlluminationModel::PBR) {
      return new PBRGLShader();
    } else {
      return nullptr;
    }
  }
  void shade(GLPrimitive& prim, Triangle3& t, GLMaterialBase* m, GLCamera& camera,
             std::vector<GLLight*>& lights, std::vector<GLShadowMapping*>& shadows, Color01 ambient,
             Fragments& fragments, Eigen::Matrix4d& invTransformMatrix);
};

}  // namespace qtgl