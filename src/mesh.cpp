#include "mesh.hpp"

TriangleMesh::TriangleMesh(std::string mesh_file) {
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string warn;
    std::string err;

    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err,
                                mesh_file.c_str());

    if (!warn.empty()) {
        std::cout << warn << std::endl;
    }
    if (!err.empty()) {
        std::cerr << err << std::endl;
    }
    if (!ret) {
        std::cout << "ERROR WHEN LOADING " << mesh_file << std::endl;
        exit(0);
    }

    vertices.resize(3, attrib.vertices.size() / 3);
    for (int i = 0; i < vertices.cols(); i++) {
        vertices.col(i)(0) = attrib.vertices[i * 3];
        vertices.col(i)(1) = attrib.vertices[i * 3 + 1];
        vertices.col(i)(2) = attrib.vertices[i * 3 + 2];
    }
    for (int i = 0; i < shapes[0].mesh.num_face_vertices.size(); i++) {
        if (shapes[0].mesh.num_face_vertices[i] != 3) {
            std::cout << "NOT TRIANGLE MESH: " << mesh_file << std::endl;
            exit(0);
        }
        tinyobj::index_t idx0 = shapes[0].mesh.indices[i * 3 + 0];
        tinyobj::index_t idx1 = shapes[0].mesh.indices[i * 3 + 1];
        tinyobj::index_t idx2 = shapes[0].mesh.indices[i * 3 + 2];
        faces.push_back(
            Face(idx0.vertex_index, idx1.vertex_index, idx2.vertex_index));
    }
}