#pragma once

#include "scene.hpp"
#include "shader.hpp"
#include <ctime>
#include <iostream>
#include <set>

class Renderer {
  public:
    virtual void render() = 0;
    virtual void loop() = 0;
};

class Wireframe : public Renderer {
  public:
    Scene *scene;
    bool with_axis;
    Eigen::Matrix<float, 4, Eigen::Dynamic> vertices;
    std::vector<std::pair<int, int>> edges;
    Wireframe(Scene *scene_);
    void render();
    void loop();
    void draw_axis();
};

class Shading : public Renderer {
  public:
    Scene *scene;
    float *z_buffer;
    Shading(Scene *scene_);
    void render();
    void loop();
    void draw_axis();
    ~Shading() { delete z_buffer; };
};