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

    int vertice_cnt = 0;
    int normal_cnt = 0;
    std::map<int, int> vdict;
    std::map<int, int> ndict;

    for (int i = 0; i < shapes[0].mesh.num_face_vertices.size(); i++) {
        if (shapes[0].mesh.num_face_vertices[i] != 3) {
            std::cout << "NOT TRIANGLE MESH: " << mesh_file << std::endl;
            exit(0);
        }
        tinyobj::index_t idx[3];
        for (int v = 0; v < 3; v++)
            idx[v] = shapes[0].mesh.indices[i * 3 + v];

        int vidx[3];
        for (int v = 0; v < 3; v++) {
            if (vdict.count(idx[v].vertex_index) == 0)
                vdict[idx[v].vertex_index] = vertice_cnt++;
            vidx[v] = vdict[idx[v].vertex_index];
        }
        faces.push_back(Face(vidx[0], vidx[1], vidx[2]));
        if (idx[0].normal_index < 0 || idx[1].normal_index < 0 ||
            idx[2].normal_index < 0) {
            std::cout << "LACK NORMAL INFOMATION: " << mesh_file << std::endl;
            exit(0);
        }
        int nidx[3];
        for (int v = 0; v < 3; v++) {
            if (ndict.count(idx[v].normal_index) == 0)
                ndict[idx[v].normal_index] = normal_cnt++;
            nidx[v] = ndict[idx[v].normal_index];
        }
        face_normals.push_back(FaceNormal(nidx[0], nidx[1], nidx[2]));
    }
    vertices.resize(3, vertice_cnt);
    normals.resize(3, normal_cnt);
    for (auto &p : vdict) {
        for (int v = 0; v < 3; v++)
            vertices.col(p.second)(v) = attrib.vertices[p.first * 3 + v];
    }
    for (auto &p : ndict) {
        for (int v = 0; v < 3; v++)
            normals.col(p.second)(v) = attrib.normals[p.first * 3 + v];
    }
}