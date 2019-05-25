#include "superpix_helper_func.h"

#include <iostream>
#include <vector>
#include <math.h>

#include "my_point.h"
#include "path.h"
#include "curve.h"
#include "bound_box_shifter.h"
#include "splitter.h"
#include "shifter.h"
#include "polygonalizer.h"

#include "approximator.h"

using namespace std;

//Converts points to pixels (with alpha value)
// TODO this shouldn't be needed because we already have trunkated every
// double in "convert_path_to_pixels"...
vector<my_point> points_to_pixels(vector<my_point> pixel_pos){
	//uint8_t alpha = 255; // 0-255
	for (size_t i = 0; i < pixel_pos.size(); i++){
		// if pos in not in int format, use math.round to convert x,y
	}
	return pixel_pos;
}

vector<my_point> convert_path_to_pixels(vector<my_point> corners){
	vector<my_point> points;
	for (size_t i=1; i < corners.size(); i++) {
		my_point p1 = corners.at(i-1), p2 = corners.at(i);
		if ( p1.x == p2.x || p1.y == p2.y){
			continue;
		}

		int x1, y1, x2, y2;
		if (p1.x < p2.x) {
			x1 = (int) round(p1.x);
			x2 = (int) round(p2.x) - 1;
		} else {
			x1 = (int) round(p1.x) - 1;
			x2 = (int) round(p2.x);
		}
		if (p1.y < p2.y) {
			y1 = (int) round(p1.y);
			y2 = (int) round(p2.y) - 1;
		} else {
			y1 = (int) round(p1.y) - 1;
			y2 = (int) round(p2.y);
		}

		if(x1==x2 && y1==y2){
			my_point new_p;
			new_p.x = x1;
			new_p.y = y1;
			points.push_back(new_p);
		} else if(x1==x2) {
			int sy = (y1 < y2) ? 1 : -1;
			for(int y = y1; in_range(y, y1, y2); y += sy) {
				my_point new_p;
				new_p.x = x1;
				new_p.y = y;
				points.push_back(new_p);
			}
		} else if(y1==y2) {
			int sx = (x1 < x2) ? 1 : -1;
			for(int x = x1; in_range(x, x1, x2); x += sx) {
				my_point new_p;
				new_p.x = x;
				new_p.y = y1;
				points.push_back(new_p);
			}
		} else {
			//assert(false);
			int sx = (x1 < x2) ? 1 : -1;
			int sy = (y1 < y2) ? 1 : -1;
			for(int x = x1; in_range(x, x1, x2); x += sx) {
				for(int y = y1; in_range(y, y1, y2); y += sy) {
					my_point new_p;
					new_p.x = x;
					new_p.y = y;
					points.push_back(new_p);
				}
			}
		}
	}
	return points;
}

vector<my_point> get_pixelated_path_wrt_center(Path path, bool shifted, bool sorted, my_point center){
	vector<Path> step1 = split_by_monotonicity(path);

	//vector<my_point> pixels_trans;
	//for (auto sub_path : step1 ) {
	//	vector<my_point> points = get_sample_points(sub_path, 100);
	//	pixels_trans.insert(pixels_trans.end(), points.begin(), points.end());
	//}
	//return pixels_trans;

	if (step1.empty()) {
		cout << "Step1 empty\n";
	}

	vector<Path> step2 = shift_endpoints_to_pixel_centres_WRT_center(step1, shifted, true, center);

	if (step2.empty()) {
		cout << "Step2 empty\n";
	}

	vector<my_point> step3 = get_pixelated_path_WRT_center(step2, sorted, center);

	if (step3.empty()) {
		cout << "Step3 empty\n";
	}
	return step3;
}

Path merge_straight_lines(Path path){
	Path new_path = path;
	for(size_t i = new_path.get_num_segments()-2; i >= 1; i--) {
		Curve curve_before = new_path.get_curve_before_seg_idx(i);
		Curve curve_after = new_path.get_curve_after_seg_idx(i);

		my_point vec1 = curve_before.get_pN(3) - curve_before.get_pN(0);
		my_point vec2 = curve_after.get_pN(3) - curve_after.get_pN(0);
		bool between_lines = false;

		//Is vec2 in the origin (zero)
		if(is_zero(vec2)){
			between_lines = true;
		} else {
			double angle = get_angle_to(vec1, vec2);
			between_lines = curve_before.is_straight() && curve_after.is_straight()
				&& abs(angle) < 0.01;
		}

		if(between_lines) {
			new_path.remove_segment(i);
		}
	}
    return new_path;
}

Path remove_degenerate_segements(Path path){
	Path new_path;
	// Get rid of degenerate curves
	for (size_t i = 0; i < path.get_num_curves(); i++){
		Curve curve = path.get_curve(i);
		if (!curve.is_degenerate()){
			new_path.add_curve(curve);
		}
	}

	return new_path;
}

Path preprocess(Path path, bool &pixelated){
	Path clean_path1 = remove_degenerate_segements(path);
	if (clean_path1.empty()){
		pixelated = true;
		return clean_path1;
	}
	Path clean_path2 = merge_straight_lines(clean_path1);
	return clean_path2;
}

vector<my_point> get_pixelated_path(Path path, bool shifted, bool sorted){
	vector<my_point> pixels0;
	bool pixelated = false;

    Path step0 = preprocess(path, pixelated);

	if (pixelated) {
		cout << "Pixelated return\n";
		return pixels0;
	}

	if (step0.empty()) {
		vector<my_point> empty_vec;
		cout << "Step0 return\n";
		return empty_vec;
	}

	my_point center;
	//TODO does this really have to be double and not int?
	vector<double> bb_coeffs_without_AA = {10000, 40, 1, 50, 100};

	step0 = snap_shape_by_optimal_bounding_box(step0, center, bb_coeffs_without_AA);

	vector<my_point> pixels_trans = get_pixelated_path_wrt_center(step0, shifted, sorted, center);

	return pixels_trans;
}

vector<my_point> superpix_helper_func(Path path){
	bool shifted, sorted;
	shifted = sorted = true;

	vector<my_point> pixelated_path = get_pixelated_path(path, shifted, sorted);
	if (pixelated_path.empty()){
		cout << "Pixelated path is does not exists!\n";
		return pixelated_path;
	}
	vector<my_point> pixel_pos = convert_path_to_pixels(pixelated_path);

	//Convert doubles to ints (might be superfluous, look into this)
	//vector<my_point> pixels = points_to_pixels(pixel_pos);

	return pixel_pos;
}
