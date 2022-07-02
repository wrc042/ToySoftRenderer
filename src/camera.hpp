#pragma once

#include "common.hpp"
#include <Eigen/Dense>
#include <iostream>

class Camera {
  public:
    Eigen::Matrix4f extrinsic;
    Eigen::Matrix4f proj_mat;

    Camera(Eigen::Vector3f T, Eigen::Quaternionf R, float aspect_ = 1.0f,
           float fovY_ = 0.25f * PI_, float z_near_ = 0.1f,
           float z_far_ = 1000.0f)
        : fovY(fovY_), aspect(aspect_), z_near(z_near_), z_far(z_far_) {
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
    Eigen::Vector3f pos;

    float fovY, aspect;
    float z_near, z_far;
    void update_proj_mat();
    void update_extrinsic();
};