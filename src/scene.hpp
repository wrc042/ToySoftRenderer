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
          width(config["screen"]["width"].asInt()),
          height(config["screen"]["height"].asInt()),
          camera(Eigen::Vector3f::Zero(), Eigen::Quaternionf::Identity(),
                 config["screen"]["width"].asFloat() /
                     config["screen"]["height"].asFloat()) {
        Eigen::Vector3f T(config["camera"]["translation"]["x"].asFloat(),
                          config["camera"]["translation"]["y"].asFloat(),
                          config["camera"]["translation"]["z"].asFloat());
        Eigen::Quaternionf R =
            Eigen::AngleAxisf(config["camera"]["rotation"]["z"].asFloat() /
                                  180 * PI_,
                              Eigen::Vector3f::UnitZ()) *
            Eigen::AngleAxisf(config["camera"]["rotation"]["y"].asFloat() /
                                  180 * PI_,
                              Eigen::Vector3f::UnitY()) *
            Eigen::AngleAxisf(config["camera"]["rotation"]["x"].asFloat() /
                                  180 * PI_,
                              Eigen::Vector3f::UnitX());
        camera.set_pose(R, T);
        framebuf = window.framebuf;
        window.set_camera(&camera);
        for (auto object : config["objects"]) {
            objects.push_back(TriangleMesh(object["path"].asString()));
        }
    }
    void add_object(std::string mesh_file) {
        objects.push_back(TriangleMesh(mesh_file));
    }
};