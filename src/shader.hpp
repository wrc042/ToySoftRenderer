#pragma once

#include "render.hpp"

struct Payload {
    Scene *scene;
    TriangleMesh *object;
    Eigen::Vector3f *verts;
    Eigen::Vector3f *verts_ca;
    float t0, t1;
    int idx;
    Payload(Scene *scene_, TriangleMesh *object_, Eigen::Vector3f *verts_,
            Eigen::Vector3f *verts_ca_, float t0_, float t1_, int idx_)
        : scene(scene_), object(object_), verts(verts_), verts_ca(verts_ca_),
          t0(t0_), t1(t1_), idx(idx_) {}
};

Eigen::Vector3f general_phong_shader(const Payload &payload);