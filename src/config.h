#pragma once

#include <json.h>

namespace Config {

Json::Value get(const std::string& path);
std::string get(const std::string& path, const std::string& def);
int64_t get(const std::string& path, int64_t def);

int pin(const std::string& name); // Return GPIO index for named pin. Take care about HAT/HAB mode. Returns -1 if no such pin defined

void load_config_file(const char*);

} // namespace Config