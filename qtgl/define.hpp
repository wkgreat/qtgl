#pragma once

#include <Eigen/Dense>
#include <limits>
#include <map>
#include <random>
#include <vector>
#include "mathutils.hpp"

namespace qtgl {

using Vertice = Eigen::Vector4d;
using Vertices = Eigen::Matrix<double, Eigen::Dynamic, 4>;
using Vertice2 = Eigen::Vector2d;
using Index3 = Eigen::Vector3i;
using Indices3 = Eigen::Matrix<int, Eigen::Dynamic, 3>;
using Normal = Eigen::Vector3d;
using Normals = Eigen::Matrix<double, Eigen::Dynamic, 3>;
using NormIndex = Eigen::Vector3i;
using NormIndices = Eigen::Matrix<int, Eigen::Dynamic, 3>;
using TexCoord = Eigen::Vector2d;
using TexCoords = Eigen::Matrix<double, Eigen::Dynamic, 2>;

struct Color {
  short R;
  short G;
  short B;
  short A;
  static Color random() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<short> distr(0, 255);
    return {distr(gen), distr(gen), distr(gen), distr(gen)};
  }
};

using Color01 = Eigen::Vector4d;

struct Color01Utils {
  static double red(Color01& c) { return c[0]; }
  static double green(Color01& c) { return c[1]; }
  static double blue(Color01& c) { return c[2]; }
  static double alpha(Color01& c) { return c[3]; }
  static Color01 clamp(Color01& c) {
    Color01 r;
    r[0] = MathUtils::limit(c[0], 0, 1);
    r[1] = MathUtils::limit(c[1], 0, 1);
    r[2] = MathUtils::limit(c[2], 0, 1);
    r[3] = MathUtils::limit(c[3], 0, 1);
    return r;
  }
  static Color01 random(bool alpha) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> real_distrib(0.0, 1.0);
    Color01 r;
    r[0] = real_distrib(gen);
    r[1] = real_distrib(gen);
    r[2] = real_distrib(gen);
    r[3] = alpha ? real_distrib(gen) : 1;
    return r;
  }
};

class Triangle {
 private:
  double f_alpha, f_beta, f_gamma;
  double _x0, _y0, _z0, _x1, _y1, _z1, _x2, _y2, _z2;
  double f01(double x, double y) {
    return (_y0 - _y1) * x + (_x1 - _x0) * y + _x0 * _y1 - _x1 * _y0;
  }
  double f12(double x, double y) {
    return (_y1 - _y2) * x + (_x2 - _x1) * y + _x1 * _y2 - _x2 * _y1;
  }
  double f20(double x, double y) {
    return (_y2 - _y0) * x + (_x0 - _x2) * y + _x2 * _y0 - _x0 * _y2;
  }

 public:
  Triangle(double x0, double y0, double z0, double x1, double y1, double z1, double x2, double y2,
           double z2)
      : _x0(x0), _y0(y0), _z0(z0), _x1(x1), _y1(y1), _z1(z1), _x2(x2), _y2(y2), _z2(z2) {
    f_alpha = f12(x0, y0);
    f_beta = f20(x1, y1);
    f_gamma = f01(x2, y2);
  }
  Triangle(Vertice& p0, Vertice& p1, Vertice& p2) {
    this->_x0 = p0[0];
    this->_y0 = p0[1];
    this->_z0 = p0[2];
    this->_x1 = p1[0];
    this->_y1 = p1[1];
    this->_z1 = p1[2];
    this->_x2 = p2[0];
    this->_y2 = p2[1];
    this->_z2 = p2[2];
    f_alpha = f12(_x0, _y0);
    f_beta = f20(_x1, _y1);
    f_gamma = f01(_x2, _y2);
  }

  double x0() const { return _x0; }
  double y0() const { return _y0; }
  double z0() const { return _z0; }
  double x1() const { return _x1; }
  double y1() const { return _y1; }
  double z1() const { return _z1; }
  double x2() const { return _x2; }
  double y2() const { return _y2; }
  double z2() const { return _z2; }

  // 重心坐标
  struct BarycentricCoordnates {
    double alpha;
    double beta;
    double gamma;
  };

  // 求解重心坐标
  BarycentricCoordnates resovleBarycentricCoordnates(double x, double y) {
    BarycentricCoordnates coord;
    coord.alpha = f12(x, y) / f_alpha;
    coord.beta = f20(x, y) / f_beta;
    coord.gamma = f01(x, y) / f_gamma;
    return coord;
  }
};

class Texture;

class Triangle2 {
 private:
  Vertice wp0, wp1, wp2;  // world position
  Vertice p0, p1, p2;     // screen position
  Vertice hp0, hp1, hp2;  // screen position with w = 1
  Normal n0, n1, n2;      // normal
  TexCoord t0, t1, t2;    // texcoord
  bool hastexture = false;

  // for resolving barycentric coordinates
  Vertice2 a, b, c;
  Eigen::Vector2d v0, v1;
  double d00, d01, d11, denom;

 public:
  Triangle2(Vertice& p0, Vertice& p1, Vertice& p2, Normal& n0, Normal& n1, Normal& n2) {
    this->p0 = p0;
    this->p1 = p1;
    this->p2 = p2;

    this->n0 = n0;
    this->n1 = n1;
    this->n2 = n2;

    this->hp0 = p0 / p0[3];
    this->hp1 = p1 / p1[3];
    this->hp2 = p2 / p2[3];

    this->a = this->hp0.head(2);
    this->b = this->hp1.head(2);
    this->c = this->hp2.head(2);

    hastexture = false;

    // paramenters for barycentric coordinatres resolving
    this->v0 = b - a;
    this->v1 = c - a;
    this->d00 = this->v0.dot(this->v0);
    this->d01 = this->v0.dot(this->v1);
    this->d11 = this->v1.dot(this->v1);
    this->denom = this->d00 * this->d11 - this->d01 * this->d01;
  }
  Triangle2(Vertice& p0, Vertice& p1, Vertice& p2, Normal& n0, Normal& n1, Normal& n2, TexCoord& t0,
            TexCoord& t1, TexCoord& t2)
      : Triangle2(p0, p1, p2, n0, n1, n2) {
    this->t0 = t0;
    this->t1 = t1;
    this->t2 = t2;
    hastexture = true;
  }

  inline double hx0() const { return hp0[0]; }
  inline double hy0() const { return hp0[1]; }
  inline double hz0() const { return hp0[2]; }

  inline double hx1() const { return hp1[0]; }
  inline double hy1() const { return hp1[1]; }
  inline double hz1() const { return hp1[2]; }

  inline double hx2() const { return hp2[0]; }
  inline double hy2() const { return hp2[1]; }
  inline double hz2() const { return hp2[2]; }

  Normal& getNormal0() { return n0; }
  Normal& getNormal1() { return n1; }
  Normal& getNormal2() { return n2; }

  bool getHasTexture() const { return hastexture; }

  TexCoord& getTexCoord0() { return t0; }
  TexCoord& getTexCoord1() { return t1; }
  TexCoord& getTexCoord2() { return t2; }

  void setWorldPos(Vertice& wp0, Vertice& wp1, Vertice& wp2) {
    this->wp0 = wp0;
    this->wp1 = wp1;
    this->wp2 = wp2;
  }

  Vertice& getWorldPos0() { return this->wp0; }
  Vertice& getWorldPos1() { return this->wp1; }
  Vertice& getWorldPos2() { return this->wp2; }

  double w0() const { return p0[3]; }
  double w1() const { return p1[3]; }
  double w2() const { return p2[3]; }

  // 重心坐标
  struct BarycentricCoordnates {
    double alpha;
    double beta;
    double gamma;
  };

  // https://gamedev.stackexchange.com/a/23745
  BarycentricCoordnates resovleBarycentricCoordnates(Vertice2 p) {
    BarycentricCoordnates coord;
    Eigen::Vector2d v2 = p - this->a;
    double d20 = v2.dot(this->v0);
    double d21 = v2.dot(this->v1);
    coord.beta = (d11 * d20 - d01 * d21) / denom;
    coord.gamma = (d00 * d21 - d01 * d20) / denom;
    coord.alpha = 1.0 - coord.beta - coord.gamma;
    return coord;
  }
};

class Triangle3 {
 private:
  Vertice wp0, wp1, wp2;                             // world position
  Vertice p0, p1, p2;                                // screen position
  Vertice hp0, hp1, hp2;                             // screen position with w = 1
  Normal n0, n1, n2;                                 // normal
  std::map<int, std::vector<TexCoord>> texCoordMap;  // texCoords

  // for resolving barycentric coordinates
  Vertice2 a, b, c;
  Eigen::Vector2d v0, v1;
  double d00, d01, d11, denom;

 public:
  Triangle3(Vertice& p0, Vertice& p1, Vertice& p2, Normal& n0, Normal& n1, Normal& n2) {
    this->p0 = p0;
    this->p1 = p1;
    this->p2 = p2;

    this->n0 = n0;
    this->n1 = n1;
    this->n2 = n2;

    this->hp0 = p0 / p0[3];
    this->hp1 = p1 / p1[3];
    this->hp2 = p2 / p2[3];

    this->a = this->hp0.head(2);
    this->b = this->hp1.head(2);
    this->c = this->hp2.head(2);

    // paramenters for barycentric coordinatres resolving
    this->v0 = b - a;
    this->v1 = c - a;
    this->d00 = this->v0.dot(this->v0);
    this->d01 = this->v0.dot(this->v1);
    this->d11 = this->v1.dot(this->v1);
    this->denom = this->d00 * this->d11 - this->d01 * this->d01;
  }

  void addTexCoords(int k, TexCoord& c0, TexCoord& c1, TexCoord& c2) {
    texCoordMap[k] = {c0, c1, c2};
  }
  void addTexCoords(TexCoord& c0, TexCoord& c1, TexCoord& c2) { addTexCoords(0, c0, c1, c2); }
  bool hasTexCoords(int k) { return texCoordMap.find(k) != texCoordMap.end(); }
  std::vector<TexCoord>& getTexCoords(int k) { return texCoordMap[k]; };
  std::vector<TexCoord>& getTexCoords() { return getTexCoords(0); };
  TexCoord& getTexCoord0(int k) { return texCoordMap[k][0]; }
  TexCoord& getTexCoord1(int k) { return texCoordMap[k][1]; }
  TexCoord& getTexCoord2(int k) { return texCoordMap[k][2]; }

  inline double hx0() const { return hp0[0]; }
  inline double hy0() const { return hp0[1]; }
  inline double hz0() const { return hp0[2]; }

  inline double hx1() const { return hp1[0]; }
  inline double hy1() const { return hp1[1]; }
  inline double hz1() const { return hp1[2]; }

  inline double hx2() const { return hp2[0]; }
  inline double hy2() const { return hp2[1]; }
  inline double hz2() const { return hp2[2]; }

  Normal& getNormal0() { return n0; }
  Normal& getNormal1() { return n1; }
  Normal& getNormal2() { return n2; }

  void setWorldPos(Vertice& wp0, Vertice& wp1, Vertice& wp2) {
    this->wp0 = wp0;
    this->wp1 = wp1;
    this->wp2 = wp2;
  }

  Vertice& getWorldPos0() { return this->wp0; }
  Vertice& getWorldPos1() { return this->wp1; }
  Vertice& getWorldPos2() { return this->wp2; }

  double w0() const { return p0[3]; }
  double w1() const { return p1[3]; }
  double w2() const { return p2[3]; }

  // 重心坐标
  struct BarycentricCoordnates {
    double alpha;
    double beta;
    double gamma;
  };

  // https://gamedev.stackexchange.com/a/23745
  BarycentricCoordnates resovleBarycentricCoordnates(Vertice2 p) {
    BarycentricCoordnates coord;
    Eigen::Vector2d v2 = p - this->a;
    double d20 = v2.dot(this->v0);
    double d21 = v2.dot(this->v1);
    coord.beta = (d11 * d20 - d01 * d21) / denom;
    coord.gamma = (d00 * d21 - d01 * d20) / denom;
    coord.alpha = 1.0 - coord.beta - coord.gamma;
    return coord;
  }
};

struct Fragment {
  Color01 color;
  double depth;  // z-buffer
  constexpr static double DEPTH_INF = std::numeric_limits<double>::max() / 2;
  static Fragment init() { return {{255, 255, 255, 255}, DEPTH_INF}; }
};

using Fragments = std::vector<std::vector<Fragment>>;

}  // namespace qtgl