#pragma once

#include "FL/Fl.H"
#include "FL/Fl_Image.H"
#include "FL/Fl_RGB_Image.H"
#include "FL/Fl_Window.H"
#include "FL/fl_draw.H"
#include "camera.hpp"
#include "json/json.h"
#include <iostream>

#define WHEEL_RATIO 60
#define ROTATE_RATIO 0.25f * PI_ / 180.0f
#define TRANS_RATIO 0.0005f

struct Color {
    uchar r, g, b;
    Color(uchar r_ = uchar(0), uchar g_ = uchar(0), uchar b_ = uchar(0))
        : r(r_), g(g_), b(b_) {}
};

class Window : public Fl_Window {
  public:
    uchar *framebuf;
    int width;
    int height;
    Camera *camera;
    Window(int w, int h) : Fl_Window(w, h), width(w), height(h), camera(NULL) {
        framebuf = new uchar[width * height * 3];
        show();
    }
    int handle(int event) {
        if (!camera || camera->fix) {
            return Fl_Window::handle(event);
        }
        float offset, dx, dy;
        switch (event) {
        case FL_MOUSEWHEEL:
            offset = Fl::event_dy() / (float)WHEEL_RATIO;
            camera->view_scale(offset);
            return 1;
        case FL_PUSH:
            last_x = Fl::event_x();
            last_y = Fl::event_y();
            return 1;
        case FL_DRAG:
            if (Fl::event_button() == FL_LEFT_MOUSE) {
                dx = -(Fl::event_x() - last_x) * (float)ROTATE_RATIO;
                dy = -(Fl::event_y() - last_y) * (float)ROTATE_RATIO;
                camera->view_rotate(dx, dy);
            } else if (Fl::event_button() == FL_RIGHT_MOUSE) {
                dx = -(Fl::event_x() - last_x) * (float)TRANS_RATIO;
                dy = +(Fl::event_y() - last_y) * (float)TRANS_RATIO;
                camera->view_translate(dx, dy);
            }
            last_x = Fl::event_x();
            last_y = Fl::event_y();
            return 1;
        default:
            return Fl_Window::handle(event);
        }
    }
    void set_camera(Camera *camera_) { camera = camera_; }
    void flush_screen() {
        fl_draw_image(framebuf, 0, 0, width, height);
        Fl::check();
    };
    void flush_screen_wait() {
        fl_draw_image(framebuf, 0, 0, width, height);
        Fl::wait();
    };
    void draw_point(int x, int y, Color &color) {
        int idx = (x + y * width) * 3;
        framebuf[idx + 0] = color.r;
        framebuf[idx + 1] = color.g;
        framebuf[idx + 2] = color.b;
    }
    void fill_background(Color &color) {
        for (int i = 0; i != width * height; i++) {
            framebuf[i * 3 + 0] = color.r;
            framebuf[i * 3 + 1] = color.g;
            framebuf[i * 3 + 2] = color.b;
        }
    }
    void draw_line(int x0, int y0, int x1, int y1, Color &color) {
        int dx_abs, dy_abs, p, step, flag;

        step = (((x1 - x0) > 0) ^ ((y1 - y0) > 0)) ? -1 : 1;
        dx_abs = abs(x1 - x0);
        dy_abs = abs(y1 - y0);
        flag = dy_abs < dx_abs;
        if (!flag) {
            std::swap(x0, y0);
            std::swap(x1, y1);
            std::swap(dx_abs, dy_abs);
        }
        if (x0 > x1) {
            std::swap(x0, x1);
            std::swap(y0, y1);
        }
        p = dy_abs * 2 - dx_abs;
        draw_point(flag ? x0 : y0, flag ? y0 : x0, color);
        while (x0 < x1) {
            x0 += 1;
            if (p > 0) {
                p -= dx_abs * 2;
                y0 += step;
            }
            p += dy_abs * 2;
            draw_point(flag ? x0 : y0, flag ? y0 : x0, color);
        }
    }
    ~Window() { delete framebuf; }

  private:
    int last_x;
    int last_y;
};