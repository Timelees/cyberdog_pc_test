#pragma once
#define DEFINE_string(name, default_value, description) static char* FLAGS_##name = (char*)default_value
#define DEFINE_bool(name, default_value, description) static bool FLAGS_##name = default_value
