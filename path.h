#ifndef PATH
#define PATH

#include <vector>
#include "segment.h"
#include "curve.h"

#include <eigen3/Eigen/Dense>
using namespace Eigen;

using namespace std;

class Path {
        vector<Segment> segments;
		//TODO winding is used by a built in java lib...
		//Not sure if this is used
		int winding_rule;

		bool is_valid_seg_idx(int i);

	public:
		Path();
		//Strait line between p1, p2
		Path(my_point p1, my_point p2);

		Path(vector<Segment>);

		bool empty();
		void clear();

		Path get_transformed_copy(Transform<double,2,Affine> at);
		void transform(Transform<double,2,Affine> at);

		size_t get_num_segments();
		size_t get_num_curves();

		Path get_reverse_path();

		Curve get_curve(int i);
		Curve get_curve_before_seg_idx(int i);
		Curve get_curve_after_seg_idx(int i);
		vector<Curve> get_curves();

		Segment get_segment(int i);
		Segment get_first_segment();
		Segment get_last_segment();

		my_point get_first_point();
		my_point get_last_point();
        my_point get_tangent_in_at_segment_index(int i);
        my_point get_tangent_out_at_segment_index(int i);

		void set(int i, Segment new_seg);
		void set_last_segment(Segment seg);

		vector<my_point> get_bounding_box();

		bool is_acute_angle(int i);
		bool is_closed();
		bool is_straight();
		bool is_nondifferentiable_at_segment_index(int i);

		void add_segment(Segment seg);
		void add_curve(Curve curve);

		void remove_segment(int i);
};

ostream& operator <<(ostream &o, Path &p);
ostream& operator <<(ostream &o, vector<Path> &p);

#endif
