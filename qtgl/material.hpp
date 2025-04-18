#pragma once

#include <memory>
#include "define.hpp"
#include "texture.hpp"

namespace qtgl {

enum class IlluminationModel {
  RANDOM,                  // random color
  CONSTANT,                // constant color
  LAMBERTIAN,              // lambertian shading
  LAMBERTIAN_BLINN_PHONG,  // lambertian shading and blinn-phong shading
  PBR                      // PBR Model
};

class GLMaterialBase {
 public:
  IlluminationModel model;
  void setModel(IlluminationModel model) { this->model = model; }
  IlluminationModel getModel() { return model; }
};

class GLMaterialRandom : public GLMaterialBase {
 public:
  GLMaterialRandom() { this->model = IlluminationModel::RANDOM; }
  Color01 getColor() { return Color01Utils::random(false); }
};

class GLMaterialConstant : public GLMaterialBase {
 private:
  Color01 color = {0, 0, 0, 0};

 public:
  GLMaterialConstant() { this->model = IlluminationModel::CONSTANT; }

  void setColor(Color01 color) { this->color = color; }
  Color01 getColor() { return this->color; }
};

class GLMaterial : public GLMaterialBase {
 private:
  Color01 ambient = {0, 0, 0, 0};                                // Ka
  Color01 diffuse = {0, 0, 0, 0};                                // Kd
  Color01 specular = {0, 0, 0, 0};                               // Ks
  Color01 emmisive = {0, 0, 0, 0};                               // Ke
  double specularHighlight = 0;                                  // Ns
  double opticalDensity = 0;                                     // Ni
  double dissolve = 0;                                           // d
  IlluminationModel illumination = IlluminationModel::CONSTANT;  // illum
  GLTexture* ambientTexture = nullptr;                           // map_Ka
  GLTexture* diffuseTexture = nullptr;                           // map_Kd
  double ambientTextureAlpha = 0.5;
  double diffuseTextureAlpha = 0.5;

 public:
  GLMaterial() { this->model = IlluminationModel::LAMBERTIAN; };
  ~GLMaterial() {
    if (ambientTexture) {
      delete ambientTexture;
    }
    if (diffuseTexture) {
      delete diffuseTexture;
    }
  }
  void setAmbient(Color01 color) { ambient = color; }
  void setDiffuse(Color01 color) { diffuse = color; }
  void setSpecular(Color01 color) { specular = color; }
  void setEmmisive(Color01 color) { emmisive = color; }
  void setSpecularHighlight(double d) { specularHighlight = d; }
  void setOpticalDensity(double d) { opticalDensity = d; }
  void setDissolve(double d) { dissolve = d; }
  void setIllumination(IlluminationModel model) {
    illumination = model;
    this->model = model;
    // TODO check the model must be LAMBERTIAN or LAMBERTIAN_BLINN_PHONG
  }
  void setAmbientTexture(GLTexture* texture) { ambientTexture = texture; }
  void setDiffuseTexture(GLTexture* texture) { diffuseTexture = texture; }
  void setAmbientTextureAlpha(double a) { ambientTextureAlpha = a; }
  void setDiffuseTextureAlpha(double a) { diffuseTextureAlpha = a; }

  Color01 getAmbient() const { return ambient; }
  Color01 getDiffuse() const { return diffuse; }
  Color01 getSpecular() const { return specular; }
  Color01 getEmmisive() const { return emmisive; }
  double getSpecularHighlight() const { return specularHighlight; }
  double getOpticalDensity() const { return opticalDensity; }
  double getDissolve() const { return dissolve; }
  IlluminationModel getIllumination() const { return illumination; }
  GLTexture* getAmbientTexture() const { return ambientTexture; }
  GLTexture* getDiffuseTexture() const { return diffuseTexture; }
  double getAmbientTextureAlpha() const { return ambientTextureAlpha; }
  double getDiffuseTextureAlpha() const { return diffuseTextureAlpha; }

  Color01 getAmbient(TexCoord* coord) {
    if (coord == nullptr || ambientTexture == nullptr) {
      return ambient;
    }
    return (1 - ambientTextureAlpha) * ambient +
           ambientTextureAlpha * ambientTexture->sample(*coord);
  }

  Color01 getDiffuse(TexCoord* coord) {
    if (coord == nullptr || diffuseTexture == nullptr) {
      return diffuse;
    }
    return (1 - diffuseTextureAlpha) * diffuse +
           diffuseTextureAlpha * diffuseTexture->sample(*coord);
  }
};

}  // namespace qtgl