#ifndef SORTER2
#define SORTER2

#include <vector>
#include "path.h"
#include "my_point.h"

using namespace std;

static double orderW = 1.0, distW = 3.0, spanOneW = 0.0, slopeW = 3.0;

vector<my_point> sort_path_points(Path path, vector<my_point> pixelPath, vector<bool> sorted);

#endif
