#ifndef POLYGON
#define POLYGON

#include <vector>
#include "path.h"
#include "my_point.h"

using namespace std;

vector<my_point> get_pixelated_path_WRT_center(vector<Path> paths, bool sorted, my_point center);

#endif
