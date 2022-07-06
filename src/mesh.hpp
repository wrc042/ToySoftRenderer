#pragma once

#include "image.hpp"
#include "tiny_obj_loader.h"
#include <Eigen/Dense>
#include <iostream>
#include <map>
#include <string>
#include <vector>

enum ShadingFrequency { FLAT, PHONG };

struct FaceIndex {
    int verts[3];
    FaceIndex() {}
    FaceIndex(int v0, int v1, int v2) {
        verts[0] = v0;
        verts[1] = v1;
        verts[2] = v2;
    }
    FaceIndex(const FaceIndex &face_) {
        verts[0] = face_.verts[0];
        verts[1] = face_.verts[1];
        verts[2] = face_.verts[2];
    }
    int operator[](int i) { return verts[i]; }
    FaceIndex operator+(int offset) const {
        FaceIndex tmp;
        tmp.verts[0] = verts[0] + offset;
        tmp.verts[1] = verts[1] + offset;
        tmp.verts[2] = verts[2] + offset;
        return tmp;
    }
};

class TriangleMesh {
  public:
    Eigen::Matrix<float, 3, Eigen::Dynamic> vertices;
    Eigen::Matrix<float, 2, Eigen::Dynamic> uvs;
    Eigen::Matrix<float, 3, Eigen::Dynamic> normals;
    std::vector<FaceIndex> face_verts;
    std::vector<FaceIndex> face_norms;
    std::vector<FaceIndex> face_uvs;
    Image diffuse_map;
    Image ambient_map;
    Image specular_map;
    Image normal_map;
    TriangleMesh(){};
    TriangleMesh(std::string mesh_file);
};