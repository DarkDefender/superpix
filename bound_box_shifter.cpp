#include <math.h>
#include <vector>
#include <cassert>
#include <iostream>

#include "my_point.h"
#include "path.h"

#include <eigen3/Eigen/Dense>
using namespace Eigen;

using namespace std;

// given 3.4, returns 2.5, 3.0, 3.5, 4.0, for example
vector<double> get_nearby_halves(double x0, double tol) {
	int xStart = (int) floor(x0 - tol);
	int xEnd = (int) ceil(x0 + tol);
	vector<double> vals;
	for(int x=xStart; x < xEnd; x++) {
		if(abs(x - x0) <= tol){
			vals.push_back(x + 0.0);
		}
		if(abs(x + 0.5 - x0) <= tol){
			vals.push_back(x + 0.5);
		}
	}
	return vals;
}

// given 3.4, returns 2.5, 3.5, 4.5, for example
vector<double> get_nearby_point_five_values(double x0, double tol) {
	int xStart = (int) floor(x0 - tol) - 1;
	int xEnd = (int) ceil(x0 + tol) + 1;
	vector<double> vals;
	for(int x=xStart; x<xEnd; x++) {
		if(abs(x + 0.5 - x0) <= tol){
			vals.push_back(x + 0.5);
		}
	}
	return vals;
}

double getIntervalCost(vector<double> interval, vector<double> interval0, double nAcute, vector<double> C) {
	// symmetry term
	double firstHalf = interval[1] - interval[0];
	double secondHalf = interval[2] - interval[1];

	double ratio = max(firstHalf, secondHalf) / min(firstHalf, secondHalf);

	double symmetry = pow(ratio, 2);
	// size term
	double oldSize = interval0[2] - interval0[0];
	double newSize = interval[2] - interval[0];

	double size = abs(oldSize - newSize);
	// position term
	double position = abs(interval[0] - interval0[0]) +
		abs(interval[1] - interval0[1]) +
		abs(interval[2] - interval0[2]);
	// sharpness term
	double frac = abs( fmod(interval[1],1) );
	bool onPixelcenter = abs(frac-0.5) < 0.01;
	double angle = onPixelcenter ? 0 : nAcute;
	//System.out.println(nAcute);
	// total cost
	double cost = C[0]*symmetry + C[1]*size + C[2]*position + C[3]*angle;

	// size should include both relative and absolute
	// also aspect ratio

	/*
	   System.out.println(MyroundToDec(cost,3) + " = " +
	   MyroundToDec(symmetry, 3) + " + " +
	   MyroundToDec(size, 3) + " + " +
	   MyroundToDec(position, 3) + " + " +
	   MyroundToDec(angle, 3));
	   */
	//System.out.println(nAcute + ": " + interval[0] + "," + interval[1] + ", " + interval[2]);
	return cost;
}

double get_bounding_box_cost(vector<double> newIntX, vector<double> oldIntX, vector<double> newIntY, vector<double> oldIntY,
		double Ax, double Ay, vector<double> C) {
	double costX = getIntervalCost(newIntX, oldIntX, Ax, C);
	double costY = getIntervalCost(newIntY, oldIntY, Ay, C);

	// aspect ratio cost
	double oldW = oldIntX[1] - oldIntX[0];
	double newW = newIntX[1] - newIntX[0];
	double oldH = oldIntY[1] - oldIntY[0];
	double newH = newIntY[1] - newIntY[0];

	double oldAsp = (oldW == 0 && oldH == 0) ? 1 :
		(max(oldW, oldH) / min(oldW, oldH));
	double newAsp = (newW == 0 && newH == 0) ? 1 :
		(max(newW, newH) / min(newW, newH));

	bool oldSquare = oldAsp < 1.01;
	bool newSquare = newAsp < 1.01;
	double aspect = (oldSquare && !newSquare) ? 1 : 0;

	// overall cost
	double cost = costX + costY + C[4]*aspect;
	//System.out.println(costX + ", " + costY);
	return cost;
}

vector<vector<double>> get_optimal_intervals(double x1, double x2, double y1, double y2,
		double tol, double Ax, double Ay, vector<double> C) {
	// x
	double cx = (x1 + x2)/2.0;
	vector<double> x1s = get_nearby_point_five_values(x1, tol);
	vector<double> cxs = get_nearby_halves(cx, tol);
	vector<double> x2s = get_nearby_point_five_values(x2, tol);
	vector<double> originalIntervalX = {x1, cx, x2};
	vector<double> bestIntervalX = {0,0,0};
	// y
	double cy = (y1 + y2)/2.0;
	vector<double> y1s = get_nearby_point_five_values(y1, tol);
	vector<double> cys = get_nearby_halves(cy, tol);
	vector<double> y2s = get_nearby_point_five_values(y2, tol);
	vector<double> originalIntervalY = {y1, cy, y2};
	vector<double> bestIntervalY = {0,0,0};
	// search
	double bestCost = 0;
	bool found = false;
	for(size_t ix=0; ix < x1s.size(); ix++) {
		for(size_t kx=0; kx < x2s.size(); kx++) {
			for(size_t jx=0; jx < cxs.size(); jx++) {
				if(abs( fmod(abs(0.5*x1s.at(ix) + 0.5*x2s.at(kx)), 1) - 0.5) < 0.1 &&
						abs(fmod(abs(cxs.at(jx)), 1)) < 0.1) {
					//System.out.println(x1s.get(ix) + ", " + x2s.get(kx) + ", " + cxs.get(jx));
					continue;
				}
				for(size_t iy=0; iy < y1s.size(); iy++) {
					for(size_t ky=0; ky < y2s.size(); ky++) {
						for(size_t jy=0; jy < cys.size(); jy++) {
							if(abs( fmod(abs(0.5*y1s.at(iy) + 0.5*y2s.at(ky)), 1) - 0.5) < 0.1 &&
									abs(fmod(abs(cys.at(jy)), 1)) < 0.1) {
								//System.out.println(y1s.get(iy) + ", " + y2s.get(ky) + ", " + cys.get(jy));
								continue;
							}
							vector<double> intervalX = {x1s.at(ix), cxs.at(jx), x2s.at(kx)};
							vector<double> intervalY = {y1s.at(iy), cys.at(jy), y2s.at(ky)};
							if(	intervalX[0] < intervalX[1] && intervalX[1] < intervalX[2] &&
									intervalY[0] < intervalY[1] && intervalY[1] < intervalY[2]	) {

								double cost = get_bounding_box_cost(
										intervalX, originalIntervalX,
										intervalY, originalIntervalY,
										Ax, Ay, C);
								if(!found || cost < bestCost) {
									for(int ii=0; ii<3; ii++) {
										bestIntervalX[ii] = intervalX[ii];
										bestIntervalY[ii] = intervalY[ii];
									}
									bestCost = cost;
									found = true;
								}
							}
						}
					}
				}
			}
		}
	}
	/*
	   if(!found) {
	   System.out.println(v1 + ", " + c + ", " + v2);
	   System.out.println(v1s.size());
	   System.out.println(cs.size());
	   System.out.println(v2s.size());
	   }
	   */
	assert(found);
	//System.out.println(bestCost + ", " + (bestInterval[1]-bestInterval[0]) + ", " + (bestInterval[2]-bestInterval[1]));

	vector<vector<double>> bestIntervals = {bestIntervalX, bestIntervalY};
	return bestIntervals;
}


int get_number_of_acute_angles_with_coordinate_value(Path path, char dir, double val) {
	int n = 0;
	for(size_t i=0; i < path.get_num_segments(); i++) {
		my_point p = path.get_segment(i).get_point();

		bool same_val = false;
		if(dir == 'x'){
			same_val = abs(p.x - val) < 0.1;
		} else {
			same_val = abs(p.y - val) < 0.1;
		}


		bool is_acute = path.is_acute_angle(i);
		if(same_val && is_acute){
			n++;
		}
	}
	return n;
}

vector<my_point> get_optimal_bounding_box(vector<my_point> old_bb, Path path, vector<double> C){
		double x1 = old_bb[0].x, y1 = old_bb[0].y, x2 = old_bb[1].x, y2 = old_bb[1].y;
		double tol = 1;
		int Ax = get_number_of_acute_angles_with_coordinate_value(path, 'x', (x1+x2)/2.0);
		int Ay = get_number_of_acute_angles_with_coordinate_value(path, 'y', (y1+y2)/2.0);
		vector<vector<double>> optIntervals = get_optimal_intervals(x1, x2, y1, y2, tol, Ax, Ay, C);

		vector<double> xs = optIntervals[0];
		vector<double> ys = optIntervals[1];

		vector<my_point> bb = {my_point{xs[0], ys[0]}, my_point{xs[1], ys[1]}, my_point{xs[2], ys[2]}};

		return bb;
}


double scale_value_by_interval(double val, vector<double> oldInt, vector<double> newInt) {
	double r = (val - oldInt[0])/(oldInt[1] - oldInt[0]);
	double newVal = r * (newInt[1]-newInt[0]) + newInt[0];
	//System.out.println(oldInt[0] + ", " + oldInt[1] + " --> " + newInt[0] + ", " + newInt[1]);
	//System.out.println(val + ", " + r + ", " + newVal);
	//System.out.println(val + ", " + newVal);
	return newVal;
}

Path scale_values(Path path, vector<double> oldInt, vector<double> newInt, char dir) {
	Path new_path = path;
	for(size_t i=0; i < path.get_num_segments(); i++) {
		Segment seg = path.get_segment(i);
		my_point p2 = seg.get_point();
		my_point p1 = p2 + seg.get_handle_in();
		my_point p3 = p2 + seg.get_handle_out();
		vector<my_point> p = {p1, p2, p3};
		for(int j=0; j<3; j++) {
			if(dir == 'x') {
				if(in_range(p[j].x, oldInt[0]-0.1, oldInt[1]+0.1)) {
					double newVal = scale_value_by_interval(p[j].x, oldInt, newInt);
					p[j].x = newVal;

				}
			}
			else {
				if(in_range(p[j].y, oldInt[0]-0.1, oldInt[1]+0.1)) {
					double newVal = scale_value_by_interval(p[j].y, oldInt, newInt);
					p[j].y = newVal;
				}
			}
		}
		new_path.set(i, Segment{p[1], p[0] - p[1], p[2] - p[1]});
	}
	return new_path;
}

Transform<double,2,Affine> get_bounding_box_rescale_transform(vector<my_point> oldBB, vector<my_point> newBB) {
	double	oldCentreX = 0.5*(oldBB[0].x + oldBB[1].x),
			oldCentreY = 0.5*(oldBB[0].y + oldBB[1].y),
			newCentreX = 0.5*(newBB[0].x + newBB[1].x),
			newCentreY = 0.5*(newBB[0].y + newBB[1].y),
			oldW = 0.5*(oldBB[1].x - oldBB[0].x),
			oldH = 0.5*(oldBB[1].y - oldBB[0].y),
			newW = 0.5*(newBB[1].x - newBB[0].x),
			newH = 0.5*(newBB[1].y - newBB[0].y);

	Transform<double,2,Affine> at;
	//Initialize at to identity matrix
	at.setIdentity();
	at.translate( Vector2d{newCentreX, newCentreY} );
	at.scale( Vector2d{newW/oldW, newH/oldH} );
	if(oldW==0 || oldH==0){
		cout << "box_rescale_tranform, oldW|| oldH" << endl;
		cout << oldW << ", " << oldH << endl;
	}
	at.translate( Vector2d{-oldCentreX, -oldCentreY});
	return at;
}

Path snap_shape_by_optimal_bounding_box(Path path, my_point &center, vector<double> bb_coeffs){
	vector<my_point> old_bb_sides_only = path.get_bounding_box();
	vector<my_point> oldBB = {	old_bb_sides_only[0],
		(old_bb_sides_only[0] + old_bb_sides_only[1]) * 0.5,
		old_bb_sides_only[1]};
	vector<my_point> newBB = get_optimal_bounding_box(old_bb_sides_only, path, bb_coeffs);

	//cout << newBB[0] << ", " << newBB[1] << ", " << newBB[2] << endl;
	double oldBBWidth = abs(oldBB[2].x - oldBB[0].x);
	double oldBBHeight = abs(oldBB[2].y - oldBB[0].y);
	double bbWidth = abs(newBB[2].x - newBB[0].x);
	double bbHeight = abs(newBB[2].y - newBB[0].y);

	//cout << "oldBB: " << oldBBWidth << ", " << oldBBHeight << endl;
	//cout << "newBB: " << bbWidth << ", " << bbHeight << endl;
	Path newPath;
	double threshold = 0.5;
	bool minW = bbWidth < threshold || oldBBWidth < threshold;
	bool minH = bbHeight < threshold || oldBBHeight < threshold;
	if(minH) {
		double y = (int) round(oldBB[1].y + 0.5)-0.5;
		if(minW) {
			double x = (int) round(oldBB[1].x + 0.5)-0.5;
			newPath = Path{my_point{x, y}, my_point{x, y}};
			center.x = x; center.y = y;
		} else {
			newPath = Path{my_point{newBB[0].x, y}, my_point{newBB[2].x, y}};
			center.x = newBB[1].x; center.y = y;
		}
	} else {
		if(minW) {
			double x = (int) round(oldBB[1].x + 0.5)-0.5;
			newPath = Path{my_point{x, newBB[0].y}, my_point{x, newBB[2].y}};
			center.x = x; center.y = newBB[1].y;
		} else {
			newPath = path;
			bool symmX = abs((newBB[1].x - newBB[0].x) - (newBB[2].x - newBB[1].x)) < 0.01;
			bool symmY = abs((newBB[1].y - newBB[0].y) - (newBB[2].y - newBB[1].y)) < 0.01;
			if(symmX) {
				vector<my_point> newBBx = {my_point{newBB[0].x, oldBB[0].y}, my_point{newBB[2].x, oldBB[2].y}};
				Transform<double,2,Affine> atBB = get_bounding_box_rescale_transform(old_bb_sides_only, newBBx);
				//cout << "Pre transform newPath\n";
				//cout << newPath << endl;
				newPath = newPath.get_transformed_copy(atBB);
				//cout << "after transform newPath\n";
				//cout << newPath << endl;
				//cout << "symmX" << endl;
			} else {
				// left half
				vector<double> oldInt1 = {oldBB[0].x, oldBB[1].x};
				vector<double> newInt1 = {newBB[0].x, newBB[1].x};
				newPath = scale_values(newPath, oldInt1, newInt1, 'x');
				// right half
				vector<double> oldInt2 = {oldBB[1].x, oldBB[2].x};
				vector<double> newInt2 = {newBB[1].x, newBB[2].x};
				newPath = scale_values(newPath, oldInt2, newInt2, 'x');
				//cout << "not symmX" << endl;
			}
			if(symmY) {
				vector<my_point> newBBy = {my_point{oldBB[0].x, newBB[0].y}, my_point{oldBB[2].x, newBB[2].y}};
				Transform<double,2,Affine> atBB = get_bounding_box_rescale_transform(old_bb_sides_only, newBBy);
				//cout << "Pre transform newPath\n";
				//cout << newPath << endl;
				newPath = newPath.get_transformed_copy(atBB);
				//cout << "after transform newPath\n";
				//cout << newPath << endl;
				//cout << "symmY" << endl;
			} else {
				// top half
				vector<double> oldInt3 = {oldBB[0].y, oldBB[1].y};
				vector<double> newInt3 = {newBB[0].y, newBB[1].y};
				newPath = scale_values(newPath, oldInt3, newInt3, 'y');
				// bottom half
				vector<double> oldInt4 = {oldBB[1].y, oldBB[2].y};
				vector<double> newInt4 = {newBB[1].y, newBB[2].y};
				newPath = scale_values(newPath, oldInt4, newInt4, 'y');
				//cout << "not symmY" << endl;
			}
			center.x = newBB[1].x; center.y = newBB[1].y;
		}
	}
	//System.out.println(newPath);
	return newPath;
}
