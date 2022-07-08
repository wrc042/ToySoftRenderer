#include "mesh.hpp"

TriangleMesh::TriangleMesh(std::string mesh_file) {
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string warn;
    std::string err;

    int idx = mesh_file.find_last_of("/");
    std::string mesh_root;
    if (idx > 0) {
        mesh_root = mesh_file.substr(0, idx);
    } else {
        mesh_root = "./";
    }

    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err,
                                mesh_file.c_str(), mesh_root.c_str());

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
    if (!materials.empty()) {
        if (!materials[0].diffuse_texname.empty()) {
            diffuse_map.preload_path(mesh_root + "/" +
                                     materials[0].diffuse_texname);
        }
        if (!materials[0].ambient_texname.empty()) {
            ambient_map.preload_path(mesh_root + "/" +
                                     materials[0].ambient_texname);
        }
        if (!materials[0].specular_texname.empty()) {
            specular_map.preload_path(mesh_root + "/" +
                                      materials[0].specular_texname);
        }
        if (!materials[0].bump_texname.empty()) {
            normal_map.preload_path(mesh_root + "/" +
                                    materials[0].bump_texname);
        }
    }

    int vertice_cnt = 0;
    int normal_cnt = 0;
    int uv_cnt = 0;
    std::map<int, int> vdict;
    std::map<int, int> ndict;
    std::map<int, int> uvdict;

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
        face_verts.push_back(FaceIndex(vidx[0], vidx[1], vidx[2]));
        if (idx[0].normal_index < 0) {
            std::cout << "LACK NORMAL INFOMATION: " << mesh_file << std::endl;
            exit(0);
        }
        int nidx[3];
        for (int v = 0; v < 3; v++) {
            if (ndict.count(idx[v].normal_index) == 0)
                ndict[idx[v].normal_index] = normal_cnt++;
            nidx[v] = ndict[idx[v].normal_index];
        }
        face_norms.push_back(FaceIndex(nidx[0], nidx[1], nidx[2]));
        if (idx[0].texcoord_index >= 0) {
            int uvidx[3];
            for (int v = 0; v < 3; v++) {
                if (uvdict.count(idx[v].texcoord_index) == 0)
                    uvdict[idx[v].texcoord_index] = uv_cnt++;
                uvidx[v] = uvdict[idx[v].texcoord_index];
            }
            face_uvs.push_back(FaceIndex(uvidx[0], uvidx[1], uvidx[2]));
        }
    }
    vertices.resize(3, vertice_cnt);
    normals.resize(3, normal_cnt);
    uvs.resize(2, uv_cnt);
    for (auto &p : vdict) {
        for (int v = 0; v < 3; v++)
            vertices.col(p.second)(v) = attrib.vertices[p.first * 3 + v];
    }
    for (auto &p : ndict) {
        for (int v = 0; v < 3; v++)
            normals.col(p.second)(v) = attrib.normals[p.first * 3 + v];
    }
    for (auto &p : uvdict) {
        for (int v = 0; v < 2; v++)
            uvs.col(p.second)(v) = attrib.texcoords[p.first * 2 + v];
    }
}