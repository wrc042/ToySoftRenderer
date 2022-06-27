#pragma once

#include "lodepng.h"
#include "scene.hpp"
#include <iostream>
#include <set>

void save_png(const char *filename, uchar *framebuffer, unsigned width,
              unsigned height) {
    std::vector<unsigned char> image;
    std::vector<unsigned char> png;
    image.resize(width * height * 4);
    for (unsigned y = 0; y < height; y++)
        for (unsigned x = 0; x < width; x++) {
            image[4 * width * y + 4 * x + 0] =
                framebuffer[3 * width * (y) + 3 * (x) + 0];
            image[4 * width * y + 4 * x + 1] =
                framebuffer[3 * width * (y) + 3 * (x) + 1];
            image[4 * width * y + 4 * x + 2] =
                framebuffer[3 * width * (y) + 3 * (x) + 2];
            image[4 * width * y + 4 * x + 3] = 255;
        }

    // Encode the image
    unsigned error = lodepng::encode(filename, image, width, height);

    // if there's an error, display it
    if (error)
        std::cout << "encoder error " << error << ": "
                  << lodepng_error_text(error) << std::endl;
}

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
    Wireframe(Scene *scene_) : scene(scene_) {
        int offset = 0;
        std::set<std::pair<int, int>> edge_set;
        for (auto &object : (scene->objects)) {
            int old_size = vertices.cols();
            Eigen::VectorXf tmpv(object.vertices.cols());
            tmpv.setOnes();
            vertices.resize(4, vertices.cols() + (object.vertices).cols());
            vertices.block(0, old_size, 3, object.vertices.cols()) =
                object.vertices;
            vertices.rightCols(object.vertices.cols()).bottomRows(1) =
                tmpv.transpose();
            for (auto &face : object.faces) {
                faces.push_back(face + offset);
                int v0, v1;
                std::pair<int, int> tmp_pair;
                v0 = face.verts[0] < face.verts[1] ? face.verts[0]
                                                   : face.verts[1];
                v1 = face.verts[0] < face.verts[1] ? face.verts[1]
                                                   : face.verts[0];
                tmp_pair = std::make_pair(v0, v1);
                if (edge_set.find(tmp_pair) == edge_set.end()) {
                    edge_set.insert(tmp_pair);
                    edges.push_back(tmp_pair);
                }
                v0 = face.verts[2] < face.verts[1] ? face.verts[2]
                                                   : face.verts[1];
                v1 = face.verts[2] < face.verts[1] ? face.verts[1]
                                                   : face.verts[2];
                tmp_pair = std::make_pair(v0, v1);
                if (edge_set.find(tmp_pair) == edge_set.end()) {
                    edge_set.insert(tmp_pair);
                    edges.push_back(tmp_pair);
                }
                v0 = face.verts[0] < face.verts[2] ? face.verts[0]
                                                   : face.verts[2];
                v1 = face.verts[0] < face.verts[2] ? face.verts[2]
                                                   : face.verts[0];
                tmp_pair = std::make_pair(v0, v1);
                if (edge_set.find(tmp_pair) == edge_set.end()) {
                    edge_set.insert(tmp_pair);
                    edges.push_back(tmp_pair);
                }
            }
            offset += old_size;
        }
    }
    void render() {
        scene->window.fill_background(Color(120, 120, 120));
        for (auto &edge : edges) {
            Eigen::Vector3f v0 = vertices_proj.block(0, edge.first, 3, 1) /
                                 vertices_proj(3, edge.first);
            Eigen::Vector3f v1 = vertices_proj.block(0, edge.second, 3, 1) /
                                 vertices_proj(3, edge.second);
            int x0, y0, x1, y1;
            x0 = floor((v0(0) / v0(2) + 0.5f) * scene->width);
            y0 = floor((0.5f - v0(1) / v0(2)) * scene->height);
            x1 = floor((v1(0) / v1(2) + 0.5f) * scene->width);
            y1 = floor((0.5f - v1(1) / v1(2)) * scene->height);
            if (x0 >= 0 && x0 < scene->width && x1 >= 0 && x1 < scene->width &&
                y0 >= 0 && y0 < scene->height && y1 >= 0 &&
                y1 < scene->height) {
                scene->window.draw_line(x0, y0, x1, y1, Color(255, 255, 255));
            }
        }
    }
    void loop() {
        while (1) {
            vertices_proj =
                scene->camera.proj_mat * scene->camera.extrinsic * vertices;
            render();
            scene->window.flush_screen_wait();
            if (!scene->window.shown()) {
                break;
            }
        }
    }
};