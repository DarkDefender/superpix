#ifndef SPLITTER
#define SPLITTER

#include "path.h"
#include "curve.h"
#include "segment.h"
#include <vector>

using namespace std;

vector<Path> split_by_monotonicity(Path path);

#endif
