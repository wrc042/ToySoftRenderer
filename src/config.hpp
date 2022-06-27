#pragma once

#include "json/json.h"
#include <fstream>
#include <iostream>
#include <string>

Json::Value parse_args(std::string config_file);
