#include "render.hpp"

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

Wireframe::Wireframe(Scene *scene_)
    : scene(scene_), with_axis(scene->config["render"]["axis"].asBool()) {
    vertices.resize(4, 0);
    int offset = 0;
    std::set<std::pair<int, int>> edge_set;
    for (auto &object : (scene->objects)) {
        Eigen::VectorXf tmpv(object.vertices.cols());
        tmpv.setOnes();
        Eigen::Matrix<float, 4, Eigen::Dynamic> vertices_;
        vertices_.resize(4, vertices.cols() + (object.vertices).cols());
        vertices_.leftCols(vertices.cols()) = vertices;
        vertices_.rightCols(object.vertices.cols()).topRows(3) =
            object.vertices;
        vertices_.rightCols(object.vertices.cols()).bottomRows(1) =
            tmpv.transpose();
        vertices = vertices_;
        for (auto face : object.faces) {
            face = face + offset;
            int v0, v1;
            std::pair<int, int> tmp_pair;
            v0 = face.verts[0] < face.verts[1] ? face.verts[0] : face.verts[1];
            v1 = face.verts[0] < face.verts[1] ? face.verts[1] : face.verts[0];
            tmp_pair = std::make_pair(v0, v1);
            if (edge_set.find(tmp_pair) == edge_set.end()) {
                edge_set.insert(tmp_pair);
                edges.push_back(tmp_pair);
            }
            v0 = face.verts[2] < face.verts[1] ? face.verts[2] : face.verts[1];
            v1 = face.verts[2] < face.verts[1] ? face.verts[1] : face.verts[2];
            tmp_pair = std::make_pair(v0, v1);
            if (edge_set.find(tmp_pair) == edge_set.end()) {
                edge_set.insert(tmp_pair);
                edges.push_back(tmp_pair);
            }
            v0 = face.verts[0] < face.verts[2] ? face.verts[0] : face.verts[2];
            v1 = face.verts[0] < face.verts[2] ? face.verts[2] : face.verts[0];
            tmp_pair = std::make_pair(v0, v1);
            if (edge_set.find(tmp_pair) == edge_set.end()) {
                edge_set.insert(tmp_pair);
                edges.push_back(tmp_pair);
            }
        }
        offset = vertices.cols();
    }
}

void Wireframe::draw_axis() {
    Eigen::Matrix<float, 4, 4> axis;
    Eigen::Matrix<float, 4, 4> axis_proj;
    axis << 0, 0.5, 0, 0, 0, 0, 0.5, 0, 0, 0, 0, 0.5, 1, 1, 1, 1;
    axis_proj = scene->camera.proj_mat * scene->camera.extrinsic * axis;
    Eigen::Vector3f v0 = axis_proj.block(0, 0, 3, 1) / axis_proj(3, 0);
    Eigen::Vector3f v1 = axis_proj.block(0, 1, 3, 1) / axis_proj(3, 1);
    int x0, y0, x1, y1;
    x0 = floor((0.5f + v0(0)) * scene->width);
    y0 = floor((0.5f - v0(1)) * scene->height);
    x1 = floor((0.5f + v1(0)) * scene->width);
    y1 = floor((0.5f - v1(1)) * scene->height);
    if (x0 >= 0 && x0 < scene->width && x1 >= 0 && x1 < scene->width &&
        y0 >= 0 && y0 < scene->height && y1 >= 0 && y1 < scene->height) {
        scene->window.draw_line(x0, y0, x1, y1, Color(255, 0, 0));
    }
    v1 = axis_proj.block(0, 2, 3, 1) / axis_proj(3, 2);
    x0 = floor((0.5f + v0(0)) * scene->width);
    y0 = floor((0.5f - v0(1)) * scene->height);
    x1 = floor((0.5f + v1(0)) * scene->width);
    y1 = floor((0.5f - v1(1)) * scene->height);
    if (x0 >= 0 && x0 < scene->width && x1 >= 0 && x1 < scene->width &&
        y0 >= 0 && y0 < scene->height && y1 >= 0 && y1 < scene->height) {
        scene->window.draw_line(x0, y0, x1, y1, Color(0, 255, 0));
    }
    v1 = axis_proj.block(0, 3, 3, 1) / axis_proj(3, 3);
    x0 = floor((0.5f + v0(0)) * scene->width);
    y0 = floor((0.5f - v0(1)) * scene->height);
    x1 = floor((0.5f + v1(0)) * scene->width);
    y1 = floor((0.5f - v1(1)) * scene->height);
    if (x0 >= 0 && x0 < scene->width && x1 >= 0 && x1 < scene->width &&
        y0 >= 0 && y0 < scene->height && y1 >= 0 && y1 < scene->height) {
        scene->window.draw_line(x0, y0, x1, y1, Color(0, 0, 255));
    }
}

void Wireframe::render() {
    vertices_proj = scene->camera.proj_mat * scene->camera.extrinsic * vertices;
    scene->window.fill_background(Color(120, 120, 120));
    for (auto &edge : edges) {
        Eigen::Vector3f v0 = vertices_proj.block(0, edge.first, 3, 1) /
                             vertices_proj(3, edge.first);
        Eigen::Vector3f v1 = vertices_proj.block(0, edge.second, 3, 1) /
                             vertices_proj(3, edge.second);
        int x0, y0, x1, y1;
        x0 = floor((0.5f + v0(0)) * scene->width);
        y0 = floor((0.5f - v0(1)) * scene->height);
        x1 = floor((0.5f + v1(0)) * scene->width);
        y1 = floor((0.5f - v1(1)) * scene->height);
        if (x0 >= 0 && x0 < scene->width && x1 >= 0 && x1 < scene->width &&
            y0 >= 0 && y0 < scene->height && y1 >= 0 && y1 < scene->height) {
            scene->window.draw_line(x0, y0, x1, y1, Color(255, 255, 255));
        }
    }
    if (with_axis)
        draw_axis();
}

void Wireframe::loop() {
    // static clock_t last_clock = clock();
    while (1) {
        render();
        // clock_t now_clock = clock();
        // std::cout << "FPS: " << 1.0f * CLOCKS_PER_SEC / (now_clock -
        // last_clock)
        //   << std::endl;
        // last_clock = now_clock;
        scene->window.flush_screen_wait();
        if (!scene->window.shown()) {
            break;
        }
    }
}

Shading::Shading(Scene *scene_) : scene(scene_) {
    z_buffer = new float[scene->width * scene->height];
    vertices.resize(4, 0);
    int offset = 0;
    std::set<std::pair<int, int>> edge_set;
    for (auto &object : (scene->objects)) {
        Eigen::VectorXf tmpv(object.vertices.cols());
        tmpv.setOnes();
        Eigen::Matrix<float, 4, Eigen::Dynamic> vertices_;
        vertices_.resize(4, vertices.cols() + (object.vertices).cols());
        vertices_.leftCols(vertices.cols()) = vertices;
        vertices_.rightCols(object.vertices.cols()).topRows(3) =
            object.vertices;
        vertices_.rightCols(object.vertices.cols()).bottomRows(1) =
            tmpv.transpose();
        vertices = vertices_;
        Eigen::Matrix<float, 3, Eigen::Dynamic> normals_;
        normals_.resize(3, normals.cols() + (object.normals).cols());
        normals_.leftCols(normals.cols()) = normals;
        normals_.rightCols(object.normals.cols()) = object.normals;
        normals = normals_;
        for (auto face : object.faces) {
            faces.push_back(face + offset);
        }
        for (auto face_normal : object.face_normals) {
            face_normals.push_back(face_normal + offset);
        }
        offset = vertices.cols();
    }
}

void Shading::render() {
    for (int i = 0; i < scene->width * scene->height; i++)
        z_buffer[i] = -std::numeric_limits<float>::infinity();
    vertices_proj = scene->camera.proj_mat * scene->camera.extrinsic * vertices;
    vertices_ex = scene->camera.extrinsic * vertices;
    scene->window.fill_background(Color(0, 0, 0));
    for (int i = 0; i < faces.size(); i++) {
        Eigen::Vector3f verts_proj[3];
        Eigen::Vector3f verts[3];
        Eigen::Vector3f norms[3];
        for (int v = 0; v < 3; v++) {
            verts_proj[v] = vertices_proj.block(0, faces[i][v], 3, 1) /
                            vertices_proj(3, faces[i][v]);
            verts[v] = vertices_ex.block(0, faces[i][v], 3, 1) /
                       vertices_ex(3, faces[i][v]);
            norms[v] = normals.col(face_normals[i][v]);
        }
        float x0, y0, x1, y1, x2, y2;
        x0 = (0.5f + verts_proj[0](0)) * scene->width;
        y0 = (0.5f - verts_proj[0](1)) * scene->height;
        x1 = (0.5f + verts_proj[1](0)) * scene->width;
        y1 = (0.5f - verts_proj[1](1)) * scene->height;
        x2 = (0.5f + verts_proj[2](0)) * scene->width;
        y2 = (0.5f - verts_proj[2](1)) * scene->height;
        int x_min = std::max(0, int(floor(std::min(std::min(x0, x1), x2))));
        int x_max = std::min(scene->width - 1,
                             int(ceil(std::max(std::max(x0, x1), x2))));
        int y_min = std::max(0, int(floor(std::min(std::min(y0, y1), y2))));
        int y_max = std::min(scene->height - 1,
                             int(ceil(std::max(std::max(y0, y1), y2))));
        Eigen::Vector2f u(x0 - x2, y0 - y2);
        Eigen::Vector2f v(x1 - x2, y1 - y2);
        for (int i = x_min; i < x_max; i++) {
            for (int j = y_min; j < y_max; j++) {
                Eigen::Vector2f p(i + 0.5f - x2, j + 0.5f - y2);
                float t0 =
                    (p(0) * v(1) - p(1) * v(0)) / (u(0) * v(1) - u(1) * v(0));
                float t1 =
                    (p(0) * u(1) - p(1) * u(0)) / (v(0) * u(1) - v(1) * u(0));
                if ((t0 < 0.0f) || (t1 < 0.0f) || ((1 - t1 - t0) < 0.0f))
                    continue;
                float tmp = (verts[0][2] * verts[1][2] +
                             verts[1][2] * (verts[2][2] - verts[0][2]) * t0 +
                             verts[0][2] * (verts[2][2] - verts[1][2]) * t1);
                float t0_ = verts[1][2] * verts[2][2] * t0 / tmp;
                float t1_ = verts[0][2] * verts[2][2] * t1 / tmp;
                // float z_inter = t0_ * verts[0][2] + t1_ * verts[1][2] +
                //                 (1 - t0_ - t1_) * verts[2][2];
                float z_inter = t0 * verts[0][2] + t1 * verts[1][2] +
                                (1 - t0 - t1) * verts[2][2];
                if (z_inter < z_buffer[i + j * scene->width])
                    continue;
                z_buffer[i + j * scene->width] = z_inter;
            }
        }
    }
    float z_min = std::numeric_limits<float>::infinity(),
          z_max = -std::numeric_limits<float>::infinity();
    for (int i = 0; i < scene->width * scene->height; i++) {
        if (isinf(z_buffer[i]))
            continue;
        z_min = std::min(z_buffer[i], z_min);
        z_max = std::max(z_buffer[i], z_max);
    }
    float scale = z_max - z_min;
    for (int i = 0; i < scene->width * scene->height; i++) {
        if (isinf(z_buffer[i])) {
            scene->window.draw_point(i % scene->width, i / scene->width,
                                     Color(0, 0, 255));
        } else {
            int c = floor(((z_max - z_buffer[i]) / scale) * 255);
            scene->window.draw_point(i % scene->width, i / scene->width,
                                     Color(c, c, c));
        }
    }
}

void Shading::loop() {
    // static clock_t last_clock = clock();
    while (1) {
        render();
        // clock_t now_clock = clock();
        // std::cout << "FPS: " << 1.0f * CLOCKS_PER_SEC / (now_clock -
        // last_clock)
        //   << std::endl;
        // last_clock = now_clock;
        scene->window.flush_screen_wait();
        if (!scene->window.shown()) {
            break;
        }
    }
}