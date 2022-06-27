#include "camera.hpp"

void Camera::update_proj_mat() {
    Eigen::Matrix4f tmp = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f ortho = Eigen::Matrix4f::Identity();
    float t = tanf(fovY / 2) * z_near;
    float r = aspect * t;
    ortho.block(0, 3, 3, 1) << 0, 0, -(z_near + z_far) / 2;
    tmp = Eigen::Matrix4f::Identity();
    tmp(0, 0) = 1.0f / r;
    tmp(1, 1) = 1.0f / t;
    tmp(2, 2) = 2.0f / (z_far - z_near);
    ortho = tmp * ortho;
    // ortho = Eigen::Scaling(1.0f / r, 1.0f / t, 2.0f / (z_far - z_near)) *
    // ortho;
    Eigen::Matrix4f proj2ortho;
    proj2ortho << z_near, 0, 0, 0, 0, z_near, 0, 0, 0, 0, z_near + z_far,
        -z_near * z_far, 0, 0, 1, 0;
    proj_mat = ortho * proj2ortho;
}

void Camera::update_extrinsic() {
    extrinsic = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f tmp = Eigen::Matrix4f::Identity();
    extrinsic.block(0, 0, 3, 3) = rotation.inverse().matrix();
    tmp.block(0, 3, 3, 1) = translation;
    extrinsic = tmp * extrinsic;
}