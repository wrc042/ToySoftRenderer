#define TINYOBJLOADER_IMPLEMENTATION // define this in only *one* .cc
#define TINYOBJLOADER_USE_MAPBOX_EARCUT
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION

#include "config.hpp"
#include "cxxopts.hpp"
#include "mesh.hpp"
#include "render.hpp"
#include "scene.hpp"
#include "window.hpp"
#include <iostream>

int main(int argc, char **argv) {
    cxxopts::Options options("ToySoftRenderer", "Just for fun:)");
    options.add_options()("c,config", "config as .json",
                          cxxopts::value<std::string>()->default_value(
                              "config.json"))("h,help", "print usage");
    auto args = options.parse(argc, argv);
    if (args["help"].count() == 0) {
        Json::Value config = parse_args(args["config"].as<std::string>());
        Scene scene(config);
        if (config["render"]["mode"].asString() == "wireframe") {
            Renderer *renderer = new Wireframe(&scene);
            renderer->loop();
        } else if (config["render"]["mode"].asString() == "shading") {
            Renderer *renderer = new Shading(&scene);
            renderer->loop();
        } else {
            std::cout << "unrecognized renderer" << std::endl;
        }
    } else {
        std::cout << options.help() << std::endl;
    }

    return 0;
}