#pragma once

#include <apriltag.h>
#include <map>
#include <string>

extern const std::map<std::string, apriltag_family_t *(*)(void)> tag_create;

extern const std::map<std::string, void (*)(apriltag_family_t*)> tag_destroy;
