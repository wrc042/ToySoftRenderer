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

Wireframe::Wireframe(Scene *scene_) : scene(scene_) {
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
            faces.push_back(face + offset);
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

void Wireframe::render() {
    scene->window.fill_background(Color(120, 120, 120));
    for (auto &edge : edges) {
        Eigen::Vector3f v0 = vertices_proj.block(0, edge.first, 3, 1) /
                             vertices_proj(3, edge.first);
        Eigen::Vector3f v1 = vertices_proj.block(0, edge.second, 3, 1) /
                             vertices_proj(3, edge.second);
        int x0, y0, x1, y1;
        // if ((v0(2) > -scene->camera.z_near) ||
        //     v1(2) > -scene->camera.z_near) {
        //     continue;
        // }
        x0 = floor((0.5f + v0(0)) * scene->width);
        y0 = floor((0.5f - v0(1)) * scene->height);
        x1 = floor((0.5f + v1(0)) * scene->width);
        y1 = floor((0.5f - v1(1)) * scene->height);
        if (x0 >= 0 && x0 < scene->width && x1 >= 0 && x1 < scene->width &&
            y0 >= 0 && y0 < scene->height && y1 >= 0 && y1 < scene->height) {
            scene->window.draw_line(x0, y0, x1, y1, Color(255, 255, 255));
        }
    }
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

void Wireframe::loop() {
    // static clock_t last_clock = clock();
    while (1) {
        vertices_proj =
            scene->camera.proj_mat * scene->camera.extrinsic * vertices;
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