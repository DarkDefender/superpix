#ifndef CURVE
#define CURVE

#include "my_point.h"
#include <vector>

using namespace std;

//Cubic bezier curve

class Curve {
		my_point p0,p1,p2,p3;
	public:
		Curve(my_point p0, my_point p1, my_point p2, my_point p3);

		double get_frame_len();

		bool validT(double t);

		vector<double> t_values_at_local_min_max(char);
		vector<double> t_values_at_inflection_points();

        Curve get_sub_curve(double start, double end);

		bool is_degenerate();
        bool is_straight();

		bool is_min_at0(char c);
		bool is_min_at1(char c);

		bool is_max_at0(char c);
		bool is_max_at1(char c);

		my_point get_pN(int N);
		my_point get_handle1();
		my_point get_handle2();

		my_point get_point(double n);

		vector<my_point> get_bounding_box();
		my_point get_tangent(double t);
};
#endif
