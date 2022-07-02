#pragma once

#include "lodepng.h"
#include "scene.hpp"
#include <ctime>
#include <iostream>
#include <set>

class Renderer {
  public:
    Scene *scene;
    virtual void render() = 0;
    virtual void loop() = 0;
};

class Wireframe : public Renderer {
  public:
    Scene *scene;
    Eigen::Matrix<float, 4, Eigen::Dynamic> vertices;
    Eigen::Matrix<float, 4, Eigen::Dynamic> vertices_proj;
    std::vector<Face> faces;
    std::vector<std::pair<int, int>> edges;
    Wireframe(Scene *scene_);
    void render();
    void loop();
};