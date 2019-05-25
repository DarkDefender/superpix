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

		bool is_valid_seg_idx(size_t i);

	public:
		Path();
		//Strait line between p1, p2
		Path(my_point p1, my_point p2);

		Path(vector<Segment>);

		bool empty();
		void clear();

		Path get_transformed_copy(Transform<double,2,Affine> at);
		void transform(Transform<double,2,Affine> at);

		size_t get_num_segments() const;
		size_t get_num_curves() const;

		Path get_reverse_path();

		Curve get_curve(size_t i) const;
		Curve get_curve_before_seg_idx(size_t i);
		Curve get_curve_after_seg_idx(size_t i);
		vector<Curve> get_curves() const;

		Segment get_segment(size_t i) const;
		Segment get_first_segment();
		Segment get_last_segment();

		my_point get_first_point();
		my_point get_last_point();
		my_point get_tangent_in_at_segment_index(size_t i);
		my_point get_tangent_out_at_segment_index(size_t i);

		void set(size_t i, Segment new_seg);
		void set_last_segment(Segment seg);

		vector<my_point> get_bounding_box();

		bool is_acute_angle(size_t i);
		bool is_closed();
		bool is_straight();
		bool is_nondifferentiable_at_segment_index(size_t i);

		void add_segment(Segment seg);
		void add_curve(Curve curve);

		void remove_segment(size_t i);
};

ostream& operator <<(ostream &o, const Path &p);
ostream& operator <<(ostream &o, const vector<Path> &p);

#endif
