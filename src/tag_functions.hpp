#pragma once

#include <apriltag.h>
#include <string>
#include <unordered_map>


extern const std::unordered_map<std::string, std::pair<apriltag_family_t* (*) (void), void (*)(apriltag_family_t*)>> tag_fun;
