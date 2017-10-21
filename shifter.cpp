#include "shifter.h"

#include <vector>
#include <math.h>
#include "path.h"
#include "my_point.h"

#include <eigen3/Eigen/Dense>
using namespace Eigen;

int sgn(double val) {
	return (val < 0) ? -1 : (val == 0) ? 0 : 1;
}

int round_down(double val){
	if(fmod(val,1) <= 0.5){
		return (int) floor(val);
	}
	return (int) ceil(val);
}

bool same_direction(my_point v1, my_point v2) {
	return abs(v1.x*v2.y - v1.y*v2.x) < 0.01;
}

my_point get_closest_pixel_center_WRT_center(my_point p, my_point centre) {
	double dirX = sgn(p.x - centre.x);
	double dirY = sgn(p.y - centre.y);
	double newX = (dirX > 0) ? (round_down(p.x-0.5)+0.5)
		: (round(p.x+0.5)-0.5);
	double newY = (dirY > 0) ? (round_down(p.y-0.5)+0.5)
		: (round(p.y+0.5)-0.5);
	return my_point{newX, newY};
}

void shiftEndpoints(Path &path, my_point newP1, my_point newP2, bool shiftHandles) {
	if(path.empty()){
		return;
	}
	if(shiftHandles) {
		my_point oldP1 = path.get_segment(0).get_point();
		my_point oldP2 = path.get_segment(path.get_num_segments()-1).get_point();
		double oldWidth = oldP2.x - oldP1.x, newWidth = newP2.x - newP1.x;
		double oldHeight = oldP2.y - oldP1.y, newHeight = newP2.y - newP1.y;

		Transform<double,2,Affine> at;
		//Initialize at to identity matrix
		at.setIdentity();

		double ratioWidth = (oldWidth == 0) ? 1 : newWidth/oldWidth;
		double ratioHeight = (oldHeight == 0) ? 1 : newHeight/oldHeight;

		// transforms are applied in reverse order
		at.translate( Vector2d{newP1.x, newP1.y} );
		at.scale( Vector2d{ratioWidth, ratioHeight} );
		at.translate( Vector2d{-oldP1.x, -oldP1.y} );
		path.transform(at);
	} else {
		Segment seg1 = path.get_segment(0);
		Segment seg2 = path.get_last_segment();

		seg1.set_point(newP1);
		seg2.set_point(newP2);
		path.set(0, seg1);
		path.set(path.get_num_segments()-1, seg2);
	}
}

void shift_endpoints_to_pixel_centres_WRT_center(Path &path, bool shift_handles, my_point center) {
	if(path.empty()){
		return;
	}
	my_point oldP1 = path.get_segment(0).get_point();
	my_point newP1 = get_closest_pixel_center_WRT_center(oldP1, center);
	if(abs(oldP1.x-center.x) < 0.01){
		newP1.x = center.x;
	}
	if(abs(oldP1.y-center.y) < 0.01){
		newP1.y = center.y;
	}
	my_point oldP2 = path.get_segment(path.get_num_segments()-1).get_point();
	my_point newP2 = get_closest_pixel_center_WRT_center(oldP2, center);
	if(abs(oldP2.x-center.x) < 0.01){
		newP2.x = center.x;
	}
	if(abs(oldP2.y-center.y) < 0.01){
		newP2.y = center.y;
	}
	shiftEndpoints(path, newP1, newP2, shift_handles);
	//System.out.println(oldP1 + ", " + newP1 + ", " + centre);
}

void fixTangentSlopes(vector<Path> paths, vector<Path> &shiftedPaths) {
	for(size_t i=0; i < paths.size()-1; i++) {
		Path path1 = paths.at(i);
		Path path2 = paths.at(i+1);
		my_point v1 = path1.get_last_segment().get_handle_in();
		my_point v2 = path2.get_first_segment().get_handle_out() * (-1);
		Path spath1 = shiftedPaths.at(i);
		Path spath2 = shiftedPaths.at(i+1);
		Segment seg1 = spath1.get_last_segment();
		Segment seg2 = spath2.get_first_segment();
		my_point sv1 = seg1.get_handle_in();
		my_point sv2 = seg2.get_handle_out() * (-1);
		if(same_direction(v1, v2) && !same_direction(sv1, sv2)) {
			my_point w = sv1 + sv2;
			my_point w1 = w * (length(sv1) / length(w));
			my_point w2 = w * (length(sv2) / length(w)) * (-1);
			seg1.set_handle_in(w1);
			seg2.set_handle_out(w2);
			spath1.set(spath1.get_num_segments()-1, seg1);
			spath2.set(0, seg2);

			shiftedPaths[i] = spath1;
			shiftedPaths[i+1] = spath2;
			//System.out.println(v1 + ", " + v2 + " : " + sv1 + ", " + sv2 + ", " + w1 + ", " + w2);
		}
	}
}

vector<Path> shift_endpoints_to_pixel_centres_WRT_center(vector<Path> paths, bool shift, bool shift_handles, my_point center){
	if(shift) {
		vector<Path> shiftedPaths;
		for(size_t i=0; i < paths.size(); i++) {
			shiftedPaths.push_back(paths.at(i));
			shift_endpoints_to_pixel_centres_WRT_center(shiftedPaths.at(i), shift_handles, center);
		}
		fixTangentSlopes(paths, shiftedPaths);

		return shiftedPaths;
	}
	return paths;
}
