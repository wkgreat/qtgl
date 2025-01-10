#pragma once

#include "affineutils.hpp"
#include "define.hpp"
#include "mathutils.hpp"

namespace qtgl {
/*
Heading : Yaw
Pitch : Pitch
Roll : Roll
*/
class GLCamera {
 private:
  double pos_x = 0;
  double pos_y = 0;
  double pos_z = 0;
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

  void lookAt(double fx, double fy, double fz, double tx, double ty, double tz) {
    // https://stackoverflow.com/a/33790309
    // TODO how to set z axis up
    Eigen::Vector3d d(tx - fx, ty - fy, tz - fz);
    d.normalize();
    this->pitch = asinf(-d[1]);
    this->heading = atan2f(d[0], d[2]);
    this->roll = MathUtils::toRadians(180);
    this->setPosition(fx, fy, fz);
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