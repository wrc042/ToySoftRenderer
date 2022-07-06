#pragma once

#include "common.hpp"
#include <Eigen/Dense>
#include <iostream>

class Camera {
  public:
    bool fix;
    float z_near, z_far;
    Eigen::Matrix4f extrinsic;
    Eigen::Matrix4f proj_mat;
    Eigen::Vector3f pos;

    Camera(Eigen::Vector3f T = Eigen::Vector3f::Zero(),
           Eigen::Quaternionf R = Eigen::Quaternionf::Identity(),
           float aspect_ = 1.0f, bool fix_ = false, float fovY_ = 0.25f * PI_,
           float z_near_ = 0.1f, float z_far_ = 1000.0f)
        : fovY(fovY_), aspect(aspect_), z_near(z_near_), z_far(z_far_),
          fix(fix_) {
        rotation = R;
        translation = T;
        up = rotation.matrix() * Eigen::Vector3f::UnitY();
        right = rotation.matrix() * Eigen::Vector3f::UnitX();
        dir = rotation.matrix() * -Eigen::Vector3f::UnitZ();
        pos = rotation.matrix() * translation;
        target = pos + rotation.inverse().matrix() * (-pos);
        update_extrinsic();
        update_proj_mat();
    }
    void view_rotate(const float dx, const float dy);
    void view_translate(const float dx, const float dy);
    void view_scale(const float scale_);
    void set_rotation(Eigen::Quaternionf R);
    void set_translation(Eigen::Vector3f T);
    void set_pose(Eigen::Quaternionf R, Eigen::Vector3f T);

  private:
    Eigen::Quaternionf rotation;
    Eigen::Vector3f translation;

    Eigen::Vector3f up;
    Eigen::Vector3f right;
    Eigen::Vector3f dir;

    Eigen::Vector3f target;

    float fovY, aspect;
    void update_proj_mat();
    void update_extrinsic();
};