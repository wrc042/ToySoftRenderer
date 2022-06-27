#pragma once

#include "camera.hpp"
#include "config.hpp"
#include "mesh.hpp"
#include "render.hpp"
#include "window.hpp"
#include <vector>

class Scene {
  public:
    Camera camera;
    Window window;
    int width;
    int height;
    uchar *framebuf;
    std::vector<TriangleMesh> objects;
    Scene(Json::Value config)
        : window(800, 600),
          camera(Eigen::Vector3f(0, 0, 1.5), Eigen::Quaternionf::Identity(),
                 800.0f / 600),
          width(800), height(600) {
        framebuf = window.framebuf;
        window.set_camera(&camera);
    }
    void add_object(std::string mesh_file) {
        objects.push_back(TriangleMesh(mesh_file));
    }
};