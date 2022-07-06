#include "render.hpp"

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
        for (auto face : object.face_verts) {
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
    Eigen::Matrix<float, 4, Eigen::Dynamic> verts_proj;
    scene->window.fill_background(Color(120, 120, 120));
    for (auto &edge : edges) {
        verts_proj = scene->camera.proj_mat * scene->camera.extrinsic *
                     vertices.col(edge.first);
        Eigen::Vector3f v0 = verts_proj.topRows(3) / verts_proj(3, 0);
        verts_proj = scene->camera.proj_mat * scene->camera.extrinsic *
                     vertices.col(edge.second);
        Eigen::Vector3f v1 = verts_proj.topRows(3) / verts_proj(3, 0);
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
    scene->window.show();
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
    for (auto &object : scene->objects) {
        object.diffuse_map.load();
        object.ambient_map.load();
        object.specular_map.load();
        object.normal_map.load();
    }
}

void Shading::render() {
    for (int i = 0; i < scene->width * scene->height; i++)
        z_buffer[i] = -scene->camera.z_far;
    scene->window.fill_background(Color(0, 0, 0));
    for (auto &object : scene->objects) {
        Eigen::Matrix<float, 4, Eigen::Dynamic> vertices_;
        vertices_.resize(4, object.vertices.cols());
        vertices_.topRows(3) = object.vertices;
        vertices_.bottomRows(1) = Eigen::RowVectorXf::Ones(vertices_.cols());
        for (int i = 0; i < object.face_verts.size(); i++) {
            Eigen::Vector4f verts_proj_[3];
            Eigen::Vector3f verts_proj[3];
            Eigen::Vector4f verts_ca_[3];
            Eigen::Vector3f verts_ca[3];
            Eigen::Vector3f verts[3];
            Eigen::Vector3f norms[3];
            Eigen::Vector2f uvs[3];
            for (int v = 0; v < 3; v++) {
                verts[v] = vertices_.col(object.face_verts[i][v]).topRows(3);
                verts_ca_[v] = scene->camera.extrinsic *
                               vertices_.col(object.face_verts[i][v]);
                verts_proj_[v] = scene->camera.proj_mat * verts_ca_[v];
                verts_proj[v] =
                    verts_proj_[v].topRows(3) / verts_proj_[v](3, 0);
                verts_ca[v] = verts_ca_[v].topRows(3) / verts_ca_[v](3, 0);
                norms[v] = object.normals.col(object.face_norms[i][v]);
                uvs[v](0) = object.uvs.col(object.face_uvs[i][v])(0);
                uvs[v](1) = object.uvs.col(object.face_uvs[i][v])(1);
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
                    float tmp = u(0) * v(1) - u(1) * v(0);
                    // Eigen::Vector2f p(i + 0.5f - x2, j + 0.5f - y2);
                    // float t0 = (p(0) * v(1) - p(1) * v(0)) / tmp;
                    // float t1 = (p(0) * u(1) - p(1) * u(0)) / -tmp;
                    Eigen::Vector2f p(-(j + 0.5f - y2), i + 0.5f - x2);
                    float t0 = (p.dot(v)) / tmp;
                    float t1 = (p.dot(u)) / -tmp;
                    if ((t0 < 0.0f) || (t1 < 0.0f) || ((1 - t1 - t0) < 0.0f))
                        continue;
                    tmp = (verts_ca[0][2] * verts_ca[1][2] +
                           verts_ca[1][2] * (verts_ca[2][2] - verts_ca[0][2]) *
                               t0 +
                           verts_ca[0][2] * (verts_ca[2][2] - verts_ca[1][2]) *
                               t1);
                    float t0_ = verts_ca[1][2] * verts_ca[2][2] * t0 / tmp;
                    float t1_ = verts_ca[0][2] * verts_ca[2][2] * t1 / tmp;
                    float z_inter = t0_ * verts_ca[0][2] +
                                    t1_ * verts_ca[1][2] +
                                    (1 - t0_ - t1_) * verts_ca[2][2];
                    if (z_inter < z_buffer[i + j * scene->width] ||
                        z_inter > scene->camera.z_near)
                        continue;
                    z_buffer[i + j * scene->width] = z_inter;
                    // Eigen::Vector2f uv_inter =
                    //     t0 * uvs[0] + t1 * uvs[1] + (1 - t0 - t1) * uvs[2];
                    // scene->window.draw_point(
                    //     i, j,
                    //     Color(object.diffuse_map.get_pixel_bilinear(
                    //         uv_inter(0), 1 - uv_inter(1))));
                    // Eigen::Vector3f norm_inter = t0_ * norms[0] +
                    //                              t1_ * norms[1] +
                    //                              (1 - t0_ - t1_) * norms[2];
                    // norm_inter = norm_inter.normalized().cwiseAbs();
                    // scene->window.draw_point(i, j, Color(norm_inter));
                    Eigen::Vector3f color(0, 0, 0);
                    Eigen::Vector3f kd(0.8, 0.8, 0.8);
                    Eigen::Vector3f ks(0.8, 0.8, 0.8);
                    for (auto &light : scene->lights) {
                        if (light->type == POINT) {
                            Eigen::Vector3f norm_inter =
                                (t0_ * norms[0] + t1_ * norms[1] +
                                 (1 - t0_ - t1_) * norms[2])
                                    .normalized();
                            Eigen::Vector3f vert_inter =
                                t0_ * verts[0] + t1_ * verts[1] +
                                (1 - t0_ - t1_) * verts[2];
                            Eigen::Vector3f dlight = light->pos - vert_inter;
                            Eigen::Vector3f dir_light = dlight.normalized();
                            Eigen::Vector3f dir_view =
                                (scene->camera.pos - vert_inter).normalized();
                            float r2 = dlight.dot(dlight);

                            float ang = dir_light.dot(norm_inter);
                            color += kd.cwiseProduct(((light->intensity) / r2) *
                                                     std::max(ang, 0.f));
                            Eigen::Vector3f h =
                                ((dir_light + dir_view) / 2).normalized();
                            ang = std::pow(norm_inter.dot(h), 150);
                            color += ks.cwiseProduct(((light->intensity) / r2) *
                                                     std::max(ang, 0.f));
                        }
                    }
                    scene->window.draw_point(i, j, Color(color));
                }
            }
        }
    }
}

void Shading::loop() {
    scene->window.show();
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