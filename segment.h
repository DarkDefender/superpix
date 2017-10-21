#ifndef SEGMENT
#define SEGMENT
#include "my_point.h"

#include <eigen3/Eigen/Dense>
using namespace Eigen;

//TODO implement this class fully, stub for now
class Segment {
	   my_point point{0,0};
	   //Handle vectors
	   my_point handle_in{0,0};
	   my_point handle_out{0,0};
       //TODO are these bools used? Can they be removed?
	   bool horizontal, vertical;
	public:
		Segment();
		Segment(my_point point);
		Segment(my_point point, my_point handle_in, my_point handle_out);

		void update_horz_and_vert();

		my_point get_point();
		my_point get_handle_in();
		my_point get_handle_out();

		void set_point(my_point p);
		void set_handle_in(my_point p);
		void set_handle_out(my_point p);

		void transform(Transform<double,2,Affine> at);
};

class FlaggedSegment: public Segment {
		bool flag = false;
	public:
		FlaggedSegment();
		FlaggedSegment(my_point point, my_point handle_in, my_point handle_out);
		void set_flag(bool b);
		bool get_flag();
};

#endif
