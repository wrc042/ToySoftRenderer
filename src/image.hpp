#pragma once
#include "camera.hpp"
#include "image.hpp"
#include "stb_image.h"
#include "stb_image_write.h"
#include <Eigen/Dense>
#include <iostream>
#include <string>

typedef unsigned char uchar;

struct Color {
    uchar r, g, b;
    Color(uchar r_ = uchar(0), uchar g_ = uchar(0), uchar b_ = uchar(0))
        : r(r_), g(g_), b(b_) {}
    Color(Eigen::Vector3f &c) {
        r = floor(255 * c(0));
        g = floor(255 * c(1));
        b = floor(255 * c(2));
    }
    Eigen::Vector3f to_eigen_vector() {
        return Eigen::Vector3f(1.0f * r / 255, 1.0f * g / 255, 1.0f * b / 255);
    }
};

// TODO: now it's not robust
class Image {
  public:
    int width;
    int height;
    int channel;
    std::string path;
    uchar *buf;
    Image() : buf(NULL){};
    Image(std::string path_) : path(path_), buf(NULL){};
    Image(int w, int h, int c) : width(w), height(h), channel(c) {
        auto *buf = (unsigned char *)malloc(width * height * channel);
    }
    void set_path(std::string path_) { path = path_; }
    // TODO: check there is no bug of channels
    void load() { buf = stbi_load(path.c_str(), &width, &height, &channel, 3); }
    void load(std::string path_) {
        buf = stbi_load(path_.c_str(), &width, &height, &channel, 3);
    }
    void save(std::string path_) {
        path_ = path_.substr(0, path_.find_last_of(".")) + ".png";
        stbi_write_png(path_.c_str(), width, height, channel, buf, 0);
    }
    Color get_pixel(int x, int y) {
        int idx = x + y * width;
        return Color(buf[idx * 3 + 0], buf[idx * 3 + 1], buf[idx * 3 + 2]);
    }
    Eigen::Vector3f get_pixel_bilinear(float u, float v) {
        int u0 = std::max(0, int(floor(u * width)));
        int u1 = std::min(width - 1, int(ceil(u * width)));
        int v0 = std::max(0, int(floor(v * height)));
        int v1 = std::min(height - 1, int(ceil(v * height)));
        float tu = u * width - u0;
        float tv = v * height - v0;
        Eigen::Vector3f c0 = (1 - tv) * get_pixel(u0, v0).to_eigen_vector() +
                             tv * get_pixel(u0, v1).to_eigen_vector();
        Eigen::Vector3f c1 = (1 - tv) * get_pixel(u1, v0).to_eigen_vector() +
                             tv * get_pixel(u1, v1).to_eigen_vector();
        return (1 - tu) * c0 + tu * c1;
    }
    void set_pixel(int x, int y, Color &color) {
        int idx = x + y * width;
        buf[idx * 3 + 0] = color.r;
        buf[idx * 3 + 1] = color.g;
        buf[idx * 3 + 2] = color.b;
    }
    ~Image() {
        if (buf != NULL)
            stbi_image_free(buf);
    }
};
