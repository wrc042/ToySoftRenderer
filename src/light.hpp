#pragma once

#include <Eigen/Dense>
#include <iostream>

enum LightType { POINT, AREA, SPOT };

class Light {
  public:
    LightType type;
    bool fix;
    Eigen::Vector3f pos;
    Eigen::Vector3f dir;
    Eigen::Vector3f intensity;
    Light(Eigen::Vector3f pos_, Eigen::Vector3f intensity_, LightType type_,
          bool fix_)
        : pos(pos_), intensity(intensity_), type(type_), fix(fix_) {}
    // virtual Eigen::Vector3f samlpe() = 0;
};

class PointLight : public Light {
  public:
    PointLight(Eigen::Vector3f pos_, Eigen::Vector3f intensity_,
               LightType type_, bool fix_)
        : Light(pos_, intensity_, type_,fix_){}
    // Eigen::Vector3f sample() { return pos; };
};