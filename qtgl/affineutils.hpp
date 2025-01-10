#pragma once

#include "define.hpp"

namespace qtgl {

struct AffineUtils {
  static Eigen::Matrix4d translateMtx(double x, double y, double z) {
    Eigen::Matrix4d m;
    m << 1, 0, 0, 0,  //
        0, 1, 0, 0,   //
        0, 0, 1, 0,   //
        x, y, z, 1;
    return m;
  }
  static Eigen::Matrix4d rotateXMtx(double a) {
    Eigen::Matrix4d m;
    m << 1, 0, 0, 0,            //
        0, cos(a), sin(a), 0,   //
        0, -sin(a), cos(a), 0,  //
        0, 0, 0, 1;
    return m;
  }
  static Eigen::Matrix4d rotateYMtx(double a) {
    Eigen::Matrix4d m;
    m << cos(a), 0, -sin(a), 0,  //
        0, 1, 0, 0,              //
        sin(a), 0, cos(a), 0,    //
        0, 0, 0, 1;
    return m;
  }
  static Eigen::Matrix4d rotateZMtx(double a) {
    Eigen::Matrix4d m;
    m << cos(a), sin(a), 0, 0,  //
        -sin(a), cos(a), 0, 0,  //
        0, 0, 1, 0,             //
        0, 0, 0, 1;
    return m;
  }
  static Eigen::Matrix4d scaleMtx(double x, double y, double z) {
    Eigen::Matrix4d m;
    m << x, 0, 0, 0,  //
        0, y, 0, 0,   //
        0, 0, z, 0,   //
        0, 0, 0, 1;
    return m;
  }

  static Vertices affine(Vertices& vtx, Eigen::Matrix4d mtx) { return vtx * mtx; }
  static Normals norm_affine(Normals& vtx, Eigen::Matrix3d mtx) {
    return vtx * (mtx.inverse().transpose());
  }
  static Vertices translate(Vertices& vtx, double x, double y, double z) {
    return affine(vtx, translateMtx(x, y, z));
  };
  static Normals normal_translate(Normals& norms, double x, double y, double z) { return norms; };
  static Vertices rotate_x(Vertices& vtx, double a) { return affine(vtx, rotateXMtx(a)); };
  static Normals normal_rotate_x(Normals& norms, double a) {
    return norm_affine(norms, rotateXMtx(a).block(0, 0, 3, 3));
  }
  static Vertices rotate_y(Vertices& vtx, double a) { return affine(vtx, rotateYMtx(a)); };
  static Normals normal_rotate_y(Normals& norms, double a) {
    return norm_affine(norms, rotateYMtx(a).block(0, 0, 3, 3));
  };
  static Vertices rotate_z(Vertices& vtx, double a) { return affine(vtx, rotateZMtx(a)); };
  static Normals normal_rotate_z(Normals& norms, double a) {
    return norm_affine(norms, rotateZMtx(a).block(0, 0, 3, 3));
  };
  static Vertices scale(Vertices& vtx, double x, double y, double z) {
    return affine(vtx, scaleMtx(x, y, z));
  };
  static Normals norm_scale(Normals& norms, double x, double y, double z) { return norms; };
};

}  // namespace qtgl