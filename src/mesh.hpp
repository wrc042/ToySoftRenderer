#pragma once

#include "tiny_obj_loader.h"
#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <vector>

struct Face {
    int verts[3];
    Face() {}
    Face(int v0, int v1, int v2) {
        verts[0] = v0;
        verts[1] = v1;
        verts[2] = v2;
    }
    Face(const Face &face_) {
        verts[0] = face_.verts[0];
        verts[1] = face_.verts[1];
        verts[2] = face_.verts[2];
    }
    int operator[](int i) { return verts[i]; }
    Face operator+(const Face &face_) const {
        Face tmp;
        tmp.verts[0] = verts[0] + face_.verts[0];
        tmp.verts[1] = verts[1] + face_.verts[1];
        tmp.verts[2] = verts[2] + face_.verts[2];
        return tmp;
    }
    Face operator+(int offset) const {
        Face tmp;
        tmp.verts[0] = verts[0] + offset;
        tmp.verts[1] = verts[1] + offset;
        tmp.verts[2] = verts[2] + offset;
        return tmp;
    }
};

class TriangleMesh {
  public:
    Eigen::Matrix<float, 3, Eigen::Dynamic> vertices;
    std::vector<Face> faces;
    TriangleMesh(){};
    TriangleMesh(std::string mesh_file);
};