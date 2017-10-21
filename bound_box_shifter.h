#ifndef BB_SHIFT
#define BB_SHIFT
#include <vector>
#include "path.h"

Path snap_shape_by_optimal_bounding_box(Path path, my_point &center, vector<double> bb_coeffs);

#endif
