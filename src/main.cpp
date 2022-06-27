#define TINYOBJLOADER_IMPLEMENTATION // define this in only *one* .cc

#include "config.hpp"
#include "cxxopts.hpp"
#include "mesh.hpp"
#include "render.hpp"
#include "scene.hpp"
#include "window.hpp"
#include <iostream>

int main(int argc, char **argv) {
    cxxopts::Options options("ToySoftRenderer", "Just for fun:)");
    options.add_options()(
        "c,config", "config as .json",
        cxxopts::value<std::string>()->default_value("config.json"));
    auto args = options.parse(argc, argv);
    Json::Value config = parse_args(args["config"].as<std::string>());
    Scene scene(config);
    scene.add_object("suzanne.obj");
    Renderer *renderer = new Wireframe(&scene);
    renderer->loop();
    return 0;
}