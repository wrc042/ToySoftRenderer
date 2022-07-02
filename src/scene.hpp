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
        : window(config["screen"]["width"].asInt(),
                 config["screen"]["height"].asInt()),
          camera(Eigen::Vector3f(0, 0, 6), Eigen::Quaternionf::Identity(),
                 config["screen"]["width"].asFloat() /
                     config["screen"]["height"].asFloat()),
          width(config["screen"]["width"].asInt()),
          height(config["screen"]["height"].asInt()) {
        framebuf = window.framebuf;
        window.set_camera(&camera);
    }
    void add_object(std::string mesh_file) {
        objects.push_back(TriangleMesh(mesh_file));
    }
};