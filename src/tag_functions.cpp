#include "tag_functions.hpp"

// default tag families
#include <tag16h5.h>
#include <tag25h9.h>
#include <tag36h11.h>
#include <tagCircle21h7.h>
#include <tagCircle49h12.h>
#include <tagCustom48h12.h>
#include <tagStandard41h12.h>
#include <tagStandard52h13.h>

// create and delete functions for default tags
#define TAG_CREATE(name) { #name, tag##name##_create },
#define TAG_DESTROY(name) { #name, tag##name##_destroy },

// function pointer for tag family creation / destruction
const std::map<std::string, apriltag_family_t *(*)(void)> tag_create =
{
    TAG_CREATE(36h11)
    TAG_CREATE(25h9)
    TAG_CREATE(16h5)
    TAG_CREATE(Circle21h7)
    TAG_CREATE(Circle49h12)
    TAG_CREATE(Custom48h12)
    TAG_CREATE(Standard41h12)
    TAG_CREATE(Standard52h13)
};

const std::map<std::string, void (*)(apriltag_family_t*)> tag_destroy =
{
    TAG_DESTROY(36h11)
    TAG_DESTROY(25h9)
    TAG_DESTROY(16h5)
    TAG_DESTROY(Circle21h7)
    TAG_DESTROY(Circle49h12)
    TAG_DESTROY(Custom48h12)
    TAG_DESTROY(Standard41h12)
    TAG_DESTROY(Standard52h13)
};
