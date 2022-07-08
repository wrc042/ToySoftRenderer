#include "shader.hpp"

Eigen::Vector3f general_phong_shader(const Payload &payload) {
    Eigen::Vector3f color(0, 0, 0);
    Eigen::Vector3f norms[3];
    Eigen::Vector2f uvs[3];
    for (int v = 0; v < 3; v++) {
        norms[v] = payload.object->normals.col(
            payload.object->face_norms[payload.idx][v]);
        if (payload.object->has_uv) {
            uvs[v](0) = payload.object->uvs.col(
                payload.object->face_uvs[payload.idx][v])(0);
            uvs[v](1) = payload.object->uvs.col(
                payload.object->face_uvs[payload.idx][v])(1);
        }
    }
    Eigen::Vector2f uv_inter;
    Eigen::Vector3f k_diffuse;
    Eigen::Vector3f normal_map;
    Eigen::Vector3f k_specular;
    float k_shininess;
    if (payload.object->has_uv) {
        uv_inter = payload.t0 * uvs[0] + payload.t1 * uvs[1] +
                   (1 - payload.t0 - payload.t1) * uvs[2];
        if (!payload.object->diffuse_map.empty()) {
            k_diffuse = payload.object->diffuse_map.get_pixel_bilinear(
                uv_inter(0), 1 - uv_inter(1));
        } else {
            k_diffuse = payload.object->Kd;
        }
        if (!payload.object->normal_map.empty()) {
            normal_map = payload.object->normal_map.get_pixel_bilinear(
                uv_inter(0), 1 - uv_inter(1));
        }
        if (!payload.object->specular_map.empty()) {
            k_specular = payload.object->specular_map.get_pixel_bilinear(
                uv_inter(0), 1 - uv_inter(1));
        } else {
            k_specular = payload.object->Ks;
        }
        if (!payload.object->shininess_map.empty()) {
            k_shininess = payload.object->shininess_map.get_pixel_bilinear(
                uv_inter(0), 1 - uv_inter(1))(0);
        } else {
            k_shininess = payload.object->shininess;
        }
    }
    Eigen::Vector3f norm_inter =
        (payload.t0 * norms[0] + payload.t1 * norms[1] +
         (1 - payload.t0 - payload.t1) * norms[2])
            .normalized();
    Eigen::Vector3f vert_inter =
        payload.t0 * payload.verts[0] + payload.t1 * payload.verts[1] +
        (1 - payload.t0 - payload.t1) * payload.verts[2];

    if (!payload.object->normal_map.empty()) {
        auto decode_norm = [](Eigen::Vector3f &a) {
            return a * 2 - Eigen::Vector3f::Ones();
        };

        float k_norm_map = 15;
        Eigen::Vector3f t, b, ln;
        Eigen::Matrix3f TBN;
        t[1] = std::sqrt(norm_inter[0] * norm_inter[0] +
                         norm_inter[2] * norm_inter[2]);
        t[0] = norm_inter[0] * norm_inter[1] / t[1];
        t[2] = norm_inter[2] * norm_inter[1] / t[1];
        b = norm_inter.cross(t);
        TBN << t[0], b[0], norm_inter[0], t[1], b[1], norm_inter[1], t[2], b[2],
            norm_inter[2];
        float w = payload.object->normal_map.width;
        float h = payload.object->normal_map.height;
        float u = uv_inter(0);
        float v = 1 - uv_inter(1);

        float dU =
            k_norm_map *
            (decode_norm(
                 payload.object->normal_map.get_pixel_bilinear(u + 1.0f / w, v))
                 .norm() -
             decode_norm(payload.object->normal_map.get_pixel_bilinear(u, v))
                 .norm());
        float dV =
            k_norm_map *
            (decode_norm(
                 payload.object->normal_map.get_pixel_bilinear(u, v + 1.0f / h))
                 .norm() -
             decode_norm(payload.object->normal_map.get_pixel_bilinear(u, v))
                 .norm());
        ln << -dU, -dV, 1.0f;
        norm_inter = (TBN * ln).normalized();
    }
    for (auto &light : payload.scene->lights) {
        if (light->type == POINT) {

            Eigen::Vector3f light_pos_ = light->pos;
            if (!(light->fix)) {
                light_pos_ =
                    payload.scene->camera.rotation.matrix() * light->pos +
                    payload.scene->camera.translation;
            }
            Eigen::Vector3f dlight = light_pos_ - vert_inter;
            Eigen::Vector3f dir_light = dlight.normalized();
            Eigen::Vector3f dir_view =
                (payload.scene->camera.pos - vert_inter).normalized();
            float r2 = dlight.dot(dlight);

            float angd = dir_light.dot(norm_inter);
            color += k_diffuse.cwiseProduct(((light->intensity) / r2) *
                                            std::max(angd, 0.f));
            Eigen::Vector3f mid = (dir_light + dir_view).normalized();
            float angs = std::max(0.f, norm_inter.dot(mid));
            Eigen::Vector3f specular = k_specular.cwiseProduct(
                ((light->intensity) / r2) * std::powf(angs, k_shininess));
            if (angd >= 0) {
                color += specular * angd;
            }
        }
    }
    return color;
}
