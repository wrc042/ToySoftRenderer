#pragma once

#include "tiny_obj_loader.h"
#include <Eigen/Dense>
#include <iostream>
#include <map>
#include <string>
#include <vector>

enum ShadingFrequency { FLAT, PHONG };

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
    Face operator+(int offset) const {
        Face tmp;
        tmp.verts[0] = verts[0] + offset;
        tmp.verts[1] = verts[1] + offset;
        tmp.verts[2] = verts[2] + offset;
        return tmp;
    }
};

struct FaceNormal {
    int norms[3];
    FaceNormal() {}
    FaceNormal(int n0, int n1, int n2) {
        norms[0] = n0;
        norms[1] = n1;
        norms[2] = n2;
    }
    FaceNormal(const FaceNormal &facenormal_) {
        norms[0] = facenormal_.norms[0];
        norms[1] = facenormal_.norms[1];
        norms[2] = facenormal_.norms[2];
    }
    int operator[](int i) { return norms[i]; }
    FaceNormal operator+(int offset) const {
        FaceNormal tmp;
        tmp.norms[0] = norms[0] + offset;
        tmp.norms[1] = norms[1] + offset;
        tmp.norms[2] = norms[2] + offset;
        return tmp;
    }
};

class TriangleMesh {
  public:
    Eigen::Matrix<float, 3, Eigen::Dynamic> vertices;
    Eigen::Matrix<float, 3, Eigen::Dynamic> normals;
    std::vector<Face> faces;
    std::vector<FaceNormal> face_normals;
    TriangleMesh(){};
    TriangleMesh(std::string mesh_file);
};