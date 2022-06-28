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