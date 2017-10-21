#include "sorter.h"

#include <cassert>
#include <math.h>

#include "path.h"
#include "curve.h"

using namespace std;

double get_signed_line_error(double x, double y, double x0, double y0, double x1, double y1) {
	double error = (x1-x0)*(y-y0) - (x-x0)*(y1-y0);
	return error;
}

double get_signed_line_error(my_point p, my_point a, my_point b) {
	return get_signed_line_error(p.x, p.y, a.x, a.y, b.x, b.y);
}

int get_sort_direction(Curve curve) {
	my_point P0 = curve.get_pN(0), P1 = curve.get_pN(1), P2 = curve.get_pN(2), P3 = curve.get_pN(3);
	double e1 = get_signed_line_error(P1, P0, P3);
	double e2 = get_signed_line_error(P2, P0, P3);
	bool sameDir = e1*e2 > 0;
	if(!sameDir){
		return 0; // don't sort
	}
	if(e1 < 0){
		return -1; // increasing
	}
	return 1; // decreasing
}

int get_sort_direction(Path path) {
	assert(path.get_num_curves() > 0);

	Curve firstCurve = path.get_curve(0);
	Curve lastCurve = path.get_curve(path.get_num_curves()-1);

	if(isnan(firstCurve.get_pN(0).x) || isnan(firstCurve.get_pN(0).y)) {
		//System.out.println("getSortDirection");
		//System.out.println(firstCurve.getP0());
	}
	Curve curve = Curve{firstCurve.get_pN(0), firstCurve.get_pN(1),
			lastCurve.get_pN(2), lastCurve.get_pN(3)};
	return get_sort_direction(curve);
}
