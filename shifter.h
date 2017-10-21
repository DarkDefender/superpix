#ifndef SHIFTER
#define SHIFTER

#include <vector>
#include "path.h"
#include "my_point.h"

vector<Path> shift_endpoints_to_pixel_centres_WRT_center(vector<Path> paths, bool shift, bool shift_handles, my_point center);

#endif
