#include "config.hpp"

// Recursively copy the values of b into a. Both a and b must be objects.
// https://stackoverflow.com/questions/22512420/is-there-an-elegant-way-to-cascade-merge-two-json-trees-using-jsoncpp
// void update(Json::Value &a, Json::Value &b) {
//     if (!a.isObject() || !b.isObject())
//         return;

//     for (const auto &key : b.getMemberNames()) {
//         if (a[key].isObject()) {
//             update(a[key], b[key]);
//         } else {
//             a[key] = b[key];
//         }
//     }
// }
void update(Json::Value &a, Json::Value &b) {
    if (!b.isObject())
        return;

    for (const auto &key : b.getMemberNames()) {
        if (a[key].isObject()) {
            update(a[key], b[key]);
        } else if (a[key].isNull()) {
            a[key] = b[key];
        }
    }
}

Json::Value parse_args(std::string config_file = "config.json") {
    Json::Value config;
    Json::Value config_std;
    std::ifstream ifs;
    ifs.open(config_file.c_str());

    if (!ifs.is_open()) {
        std::cout << "ERROR WHEN READING CONFIG" << std::endl;
        exit(0);
    }

    if (!Json::Reader().parse(ifs, config)) {
        std::cout << "ERROR WHEN PARSING CONFIG" << std::endl;
        exit(0);
    }

    ifs.close();

    config_std["screen"]["height"] = 600;
    config_std["screen"]["width"] = 800;
    config_std["camera"]["translation"]["x"] = 0;
    config_std["camera"]["translation"]["y"] = 0;
    config_std["camera"]["translation"]["z"] = 6;
    config_std["camera"]["rotation"]["x"] = 0;
    config_std["camera"]["rotation"]["y"] = 0;
    config_std["camera"]["rotation"]["z"] = 0;

    update(config, config_std);

    // std::ofstream ofs;
    // ofs.open("config.json");
    // ofs << Json::StyledWriter().write(config_std);
    // ofs.close();

    return config;
}