#pragma once

#include "json/json.h"
#include "common.hpp"
#include <fstream>
#include <iostream>
#include <string>

Json::Value parse_args(std::string config_file);
