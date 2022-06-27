#pragma once

#include "common.hpp"
#include <Eigen/Dense>

class Camera {
  public:
    Eigen::Quaternionf rotation;
    Eigen::Vector3f translation;

    Eigen::Vector3f up;
    Eigen::Vector3f right;
    Eigen::Vector3f dir;

    Eigen::Vector3f target;
    Eigen::Vector3f pos;

    float fovY, aspect;
    float z_near, z_far;

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
        target = rotation.matrix() * (-pos) + pos;
        update_extrinsic();
        update_proj_mat();
    }
    void rotate(const float dx, const float dy) {
        float radius = (pos - target).norm();
        rotation =
            Eigen::AngleAxisf(dx, up) * Eigen::AngleAxisf(dy, right) * rotation;
        up = rotation.matrix() * Eigen::Vector3f::UnitY();
        right = rotation.matrix() * Eigen::Vector3f::UnitX();
        dir = rotation.matrix() * -Eigen::Vector3f::UnitZ();
        pos = target - dir * radius;
        translation = rotation.inverse().matrix() * pos;
        update_extrinsic();
    }
    void translate(const float dx, const float dy) {
        float radius = (pos - target).norm();
        target += radius * (-dy * up + dx * right);
        pos = target - dir * radius;
        translation = rotation.inverse().matrix() * pos;
        update_extrinsic();
    }
    void scale(const float scale_) {
        float radius = (pos - target).norm();
        radius *= std::exp(scale_);
        pos = target - dir * radius;
        translation = rotation.inverse().matrix() * pos;
        update_extrinsic();
    }

  private:
    void update_proj_mat();
    void update_extrinsic();
};