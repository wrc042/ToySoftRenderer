#include "camera.hpp"

void Camera::update_proj_mat() {
    float t = tanf(fovY / 2) * z_near;
    float r = aspect * t;
    proj_mat << z_near / r, 0, 0, 0, 0, z_near / t, 0, 0, 0, 0,
        -(z_near + z_far) / (z_far - z_near),
        -2 * z_near * z_far / (z_far - z_near), 0, 0, -1, 0;
}

void Camera::update_extrinsic() {
    extrinsic = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f tmp = Eigen::Matrix4f::Identity();
    extrinsic.block(0, 0, 3, 3) = rotation.inverse().matrix();
    tmp.block(0, 3, 3, 1) = -translation;
    extrinsic = extrinsic * tmp;
}

void Camera::view_rotate(const float dx, const float dy) {
    float radius = (pos - target).norm();
    rotation =
        Eigen::AngleAxisf(dx, up) * Eigen::AngleAxisf(dy, right) * rotation;
    up = rotation.matrix() * Eigen::Vector3f::UnitY();
    right = rotation.matrix() * Eigen::Vector3f::UnitX();
    dir = rotation.matrix() * -Eigen::Vector3f::UnitZ();
    pos = target - dir * radius;
    translation = pos;
    update_extrinsic();
}

void Camera::view_translate(const float dx, const float dy) {
    float radius = (pos - target).norm();
    target += radius * (dy * up + dx * right);
    pos = target - dir * radius;
    translation = pos;
    update_extrinsic();
}

void Camera::view_scale(const float scale_) {
    float radius = (pos - target).norm();
    radius *= std::exp(scale_);
    pos = target - dir * radius;
    translation = pos;
    update_extrinsic();
}

void Camera::set_rotation(Eigen::Quaternionf R) {
    rotation = R;
    up = rotation.matrix() * Eigen::Vector3f::UnitY();
    right = rotation.matrix() * Eigen::Vector3f::UnitX();
    dir = rotation.matrix() * -Eigen::Vector3f::UnitZ();
    pos = translation;
    float radius = (pos - target).norm();
    target = pos + dir * radius;
    update_extrinsic();
};

void Camera::set_translation(Eigen::Vector3f T) {
    translation = T;
    up = rotation.matrix() * Eigen::Vector3f::UnitY();
    right = rotation.matrix() * Eigen::Vector3f::UnitX();
    dir = rotation.matrix() * -Eigen::Vector3f::UnitZ();
    pos = translation;
    float radius = (pos - target).norm();
    target = pos + dir * radius;
    update_extrinsic();
};

void Camera::set_pose(Eigen::Quaternionf R, Eigen::Vector3f T) {
    rotation = R;
    translation = T;
    up = rotation.matrix() * Eigen::Vector3f::UnitY();
    right = rotation.matrix() * Eigen::Vector3f::UnitX();
    dir = rotation.matrix() * -Eigen::Vector3f::UnitZ();
    pos = translation;
    float radius = (pos - target).norm();
    target = pos + dir * radius;
    update_extrinsic();
}
