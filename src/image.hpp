#pragma once

#include "camera.hpp"
#include "image.hpp"
#include "stb_image.h"
#include "stb_image_write.h"
#include <Eigen/Dense>
#include <iostream>
#include <string>

typedef unsigned char uchar;

struct Colorf {
    Eigen::Vector3f rgb;
    Colorf(float r = 0, float g = 0, float b = 0) { rgb << r, b, g; }
    Colorf(Eigen::Vector3f rgb_) : rgb(rgb_) {}
    Eigen::Vector3f vec() { return rgb; }
    float operator()(int i) const { return rgb(i); }
};

struct Color {
    uchar r, g, b;
    Color(uchar r_ = uchar(0), uchar g_ = uchar(0), uchar b_ = uchar(0))
        : r(r_), g(g_), b(b_) {}
    Color(const Eigen::Vector3f &c) {
        r = uchar(std::min(255, int(floor(255 * c(0)))));
        g = uchar(std::min(255, int(floor(255 * c(1)))));
        b = uchar(std::min(255, int(floor(255 * c(2)))));
    }
    Color(const Colorf &c) {
        r = uchar(floor(255 * c(0)));
        g = uchar(floor(255 * c(1)));
        b = uchar(floor(255 * c(2)));
    }
    Eigen::Vector3f vec() {
        return Eigen::Vector3f(1.0f * r / 255, 1.0f * g / 255, 1.0f * b / 255);
    }
};

class Image {
    // origin: top-left
  public:
    int width;
    int height;
    int _channel;
    std::string path;
    uchar *buf;
    Image() : buf(NULL){};
    Image(std::string path_) : path(path_), buf(NULL){};
    Image(int w, int h) : width(w), height(h) {
        buf = (unsigned char *)malloc(width * height * 3);
    }
    void preload_path(std::string path_) { path = path_; }
    // TODO: check there is no bug of channels
    void load() {
        if (path.empty()) {
            return;
        }
        buf = stbi_load(path.c_str(), &width, &height, &_channel, 3);
    }
    void load(std::string path_) {
        buf = stbi_load(path_.c_str(), &width, &height, &_channel, 3);
    }
    void save(std::string path_) {
        path_ = path_.substr(0, path_.find_last_of(".")) + ".png";
        stbi_write_png(path_.c_str(), width, height, 3, buf, 0);
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
        Eigen::Vector3f c0 =
            (1 - tv) * get_pixel(u0, v0).vec() + tv * get_pixel(u0, v1).vec();
        Eigen::Vector3f c1 =
            (1 - tv) * get_pixel(u1, v0).vec() + tv * get_pixel(u1, v1).vec();
        return (1 - tu) * c0 + tu * c1;
    }
    void fill_background(Color &color) {
        for (int i = 0; i != width * height; i++) {
            buf[i * 3 + 0] = color.r;
            buf[i * 3 + 1] = color.g;
            buf[i * 3 + 2] = color.b;
        }
    }
    void set_pixel(int x, int y, Color &color) {
        int idx = x + y * width;
        buf[idx * 3 + 0] = color.r;
        buf[idx * 3 + 1] = color.g;
        buf[idx * 3 + 2] = color.b;
    }
    bool empty() { return (buf == NULL); }
    ~Image() {
        if (buf != NULL)
            stbi_image_free(buf);
    }
};
