#ifndef APPROX
#define APPROX

#include <vector>
#include "my_point.h"
#include "path.h"

using namespace std;

vector<my_point> get_sample_points(Path path, int n);

double get_min_dist(my_point p, vector<my_point> samples);
int get_min_dist_ind(my_point p, vector<my_point> samples);

#endif
