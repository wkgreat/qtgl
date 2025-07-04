#pragma once

#include "affineutils.hpp"
#include "define.hpp"
#include "mathutils.hpp"

namespace qtgl {
/* z-up
Heading(Yaw)  绕Z轴逆时针旋转
Pitch         绕X轴逆时针旋转
Roll          绕Y轴逆时针旋转
*/
class GLCamera {
 private:
  double pos_x = 0;
  double pos_y = 0;
  double pos_z = 0;
  double to_x = 0;
  double to_y = 0;
  double to_z = 0;
  double heading = 0;
  double pitch = 0;
  double roll = 0;

 public:
  GLCamera() : pos_x(0), pos_y(0), pos_z(0), heading(0), pitch(0), roll(0) {}
  GLCamera(double pos_x, double pos_y, double pos_z, double heading, double pitch, double roll)
      : pos_x(pos_x), pos_y(pos_y), pos_z(pos_z), heading(heading), pitch(pitch), roll(roll) {}
  void setPosition(double x, double y, double z) {
    this->pos_x = x;
    this->pos_y = y;
    this->pos_z = z;
  }
  void setPosture(double heading, double pitch, double roll) {
    this->heading = heading;
    this->pitch = pitch;
    this->roll = roll;
  }
  void setPosX(double x) { this->pos_x = x; }
  void setPosY(double y) { this->pos_y = y; }
  void setPosZ(double z) { this->pos_z = z; }
  void setHeading(double heading) { this->heading = heading; }
  void setPitch(double pitch) { this->pitch = pitch; }
  void setRoll(double roll) { this->roll = roll; }
  double getPosX() const { return this->pos_x; }
  double getPosY() const { return this->pos_y; }
  double getPosZ() const { return this->pos_z; }
  double getHeading() const { return this->heading; }
  double getPitch() const { return this->pitch; }
  double getRool() const { return this->roll; }
  Vertice getPositionVertice() { return {pos_x, pos_y, pos_z, 1}; }
  double getToX() const { return this->to_x; }
  double getToY() const { return this->to_y; }
  double getToZ() const { return this->to_z; }

  void lookAt(double fx, double fy, double fz, double tx, double ty, double tz) {
    // https://stackoverflow.com/a/33790309
    // TODO how to set z axis up
    Eigen::Vector3d d(tx - fx, ty - fy, tz - fz);
    d.normalize();
    this->pitch = asinf(-d[1]);
    this->heading = atan2f(d[0], d[2]);
    this->roll = MathUtils::toRadians(180);
    this->setPosition(fx, fy, fz);
    this->to_x = tx;
    this->to_y = ty;
    this->to_z = tz;
  }

  void move(double x, double y) {
    Eigen::Matrix4d viewMtx = viewMatrix();
    Eigen::Vector3d xbasis = viewMtx.row(0).head(3);
    Eigen::Vector3d ybasis = viewMtx.row(1).head(3);
    Eigen::Vector3d from(this->pos_x, this->pos_y, this->pos_z);
    Eigen::Vector3d to(this->to_x, this->to_y, this->to_z);
    Eigen::Vector3d d = xbasis * -x + ybasis * -y;
    from = from + d;
    to = to + d;
    lookAt(from[0], from[1], from[2], to[0], to[1], to[2]);
  }

  void zoom(double factor) {
    double proj_near = 1;  // TODO near of projection
    double nearest = 1;
    factor = std::signbit(factor) ? -1.5 : 0.5;
    Eigen::Vector3d v(this->to_x - this->pos_x, this->to_y - this->pos_y, this->to_z - this->pos_z);
    double d = std::min(v.norm(), proj_near);
    if (factor > 0 && d < nearest) {
      // 太近，不在zoom in了
      return;
    }
    v = v.normalized() * d * factor;
    Eigen::Vector3d from(this->pos_x, this->pos_y, this->pos_z);
    from = from + v;
    lookAt(from[0], from[1], from[2], this->to_x, this->to_y, this->to_z);
  }

  void round(double right, double up) {
    Eigen::Vector4d from(this->pos_x, this->pos_y, this->pos_z, 1);
    Eigen::Vector4d to(this->to_x, this->to_y, this->to_z, 1);
    Eigen::Matrix4d mtx;
    Vertices m = static_cast<Vertices>(Eigen::Matrix4d::Identity());
    m = AffineUtils::translate(m, -to_x, -to_y, -to_z);
    m = AffineUtils::rotate_y(m, -heading);
    m = AffineUtils::rotate_x(m, -pitch);
    m = AffineUtils::rotate_z(m, -roll);
    mtx = static_cast<Eigen::Matrix4d>(m);
    from = from.transpose() * mtx;

    Eigen::Matrix4d mtx2;
    Vertices m2 = static_cast<Vertices>(Eigen::Matrix4d::Identity());
    m2 = static_cast<Vertices>(Eigen::Matrix4d::Identity());
    m2 = AffineUtils::rotate_x(m2, up);
    m2 = AffineUtils::rotate_y(m2, right);
    mtx2 = static_cast<Eigen::Matrix4d>(m2);
    from = from.transpose() * mtx2;

    from = from.transpose() * (mtx.inverse());
    this->lookAt(from[0], from[1], from[2], to_x, to_y, to_z);
  }

  Eigen::Matrix4d viewMatrix() {
    Eigen::Matrix4d eyeMtx = Eigen::Matrix4d::Identity();
    Vertices rotateMtx = static_cast<Vertices>(eyeMtx);
    rotateMtx = AffineUtils::rotate_y(rotateMtx, -heading);
    rotateMtx = AffineUtils::rotate_x(rotateMtx, -pitch);
    rotateMtx = AffineUtils::rotate_z(rotateMtx, -roll);
    Vertices translateMtx = static_cast<Vertices>(eyeMtx);
    translateMtx = AffineUtils::translate(translateMtx, -pos_x, -pos_y, -pos_z);
    Eigen::Matrix4d viewMtx =
        static_cast<Eigen::Matrix4d>(translateMtx) * static_cast<Eigen::Matrix4d>(rotateMtx);
    return viewMtx;
  }
};
}  // namespace qtgl