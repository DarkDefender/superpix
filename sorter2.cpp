#include "sorter2.h"

#include <vector>
#include <cassert>
#include <math.h>

#include "path.h"
#include "my_point.h"
#include "sorter.h"
#include "approximator.h"

using namespace std;

bool is_valid_span(my_point v) {
	int absX = (int) abs(round(v.x));
	int absY = (int) abs(round(v.y));
	return (absX >=1 && absY >= 1) && (absX == 1 || absY == 1);
}

vector<my_point> get_pixel_centres_in_span(my_point corner1, my_point corner2) {
	int x1 = (int) round(corner1.x);
	int y1 = (int) round(corner1.y);
	int x2 = (int) round(corner2.x);
	int y2 = (int) round(corner2.y);
	int dirX = (x1 < x2) ? 1 : -1;
	int dirY = (y1 < y2) ? 1 : -1;
	int x = x1, y = y1;
	vector<my_point> points;
	//int i = 0;
	while(in_range(x, x1, x2) && in_range(y, y1, y2)) {
		points.push_back( my_point{x+0.5*(double)dirX, y+0.5*(double)dirY});
		//i++;
		bool incX = in_range(x+dirX+0.5*(double)dirX, x1, x2);
		bool incY = in_range(y+dirY+0.5*(double)dirY, y1, y2);
		if(incX){
			x += dirX;
		}
		if(incY){
			y += dirY;
		}
		if(!incX && !incY){
			break;
		}
	}
	//System.out.println(corner1 + ", " + corner2 + " : ");
	//for(int i=0; i<points.size(); i++) {
	//	System.out.println(i + ", " + points.get(i));
	//}
	return points;
}

double get_span_error(my_point corner1, my_point corner2, vector<my_point> samples) {
	vector<my_point> pixelCentres = get_pixel_centres_in_span(corner1, corner2);
	double error = get_min_dist(pixelCentres.at(0), samples);

	for(size_t i=0; i < pixelCentres.size(); i++) {
		error = max(error, get_min_dist(pixelCentres.at(i), samples));
	}
	return error;
}

my_point get_slope_from_samples(size_t i, vector<my_point> samples) {
	assert(samples.size() >= 3);
	if (i == 0){
		return samples.at(1) - samples.at(0);
	} else if( i == samples.size()-1){
		return samples.at(samples.size()-1) - samples.at(samples.size()-2);
	}
	return samples.at(i+1) - samples.at(i-1);
}

double get_span_slope_error(my_point corner1, my_point corner2, vector<my_point> samples) {
	my_point mid = (corner1 + corner2) * 0.5;

	int minDistInd = get_min_dist_ind(mid, samples);
	my_point slope = get_slope_from_samples(minDistInd, samples);
	my_point spanSlope = corner2 - corner1;
	double angle = get_angle_to(slope, spanSlope);
	return abs(angle);
}

double get_max_span_one_length(vector<my_point> points) {
	vector<my_point> vectors;
	for(size_t i=1; i < points.size(); i++) {
		vectors.push_back(points.at(i) - points.at(i-1));
	}
	int maxLength = 0;
	int currLength = 0;
	for(size_t i=0; i < vectors.size(); i++) {
		my_point v1;
		my_point v2 = vectors.at(i);
		bool slopeOne = abs(abs(v2.x) - 1) < 0.01 && abs(abs(v2.y) - 1) < 0.01;
		bool sameAsPrev;
		if (i == 0){
			sameAsPrev = true;
		} else {
			v1 = vectors.at(i-1);
			sameAsPrev = (abs(abs(v1.x) - abs(v2.x)) < 0.01) && (abs(abs(v1.y) - abs(v2.y)) < 0.01);
		}
		if(slopeOne) {
			if(sameAsPrev){
				currLength++;
			} else {
				currLength = 1;
			}
		} else {
			currLength = 0;
		}
		if(currLength > maxLength){
			maxLength ++;
		}
	}
	if(maxLength <= 3) maxLength = 0;
	return min(maxLength,5);
}

// fraction of segments that are out of place, in terms of slope order
double get_ordering_cost_WRT_segment(vector<my_point> points, size_t k, int sortDir) {
	vector<my_point> vectors;
	for(size_t i=1; i < points.size(); i++) {
		vectors.push_back(points.at(i) - points.at(i-1));
	}
	int nTotal = 0;
	int nOutOfOrder = 0;
	my_point seg_k = vectors.at(k);
	//System.out.println(k + ", " + seg_k + ", " + sortDir + " : ");
	for(size_t i=0; i < vectors.size(); i++) {
		if(i==k){
			continue;
		}
		// vector i = point i-1 to point i
		my_point seg_i = vectors.at(i);
		double angleTo = (i < k) ? get_angle_to(seg_i, seg_k) : get_angle_to(seg_k, seg_i);
		//System.out.println(i + ", " + k + ", " + seg_i + ", " + seg_k + ", " + angleTo);
		//System.out.println(i + " // " + seg_i + ", " + angleTo);
		if(sortDir*angleTo < 0){
			nOutOfOrder++;
		}
		nTotal++;
	}
	double cost = (double) nOutOfOrder / (double) nTotal;
	//System.out.println(nOutOfOrder);
	return cost;
}

// return true if shifted
bool try_shifting_point(vector<my_point> &points, int i, int dx, int dy,
		vector<my_point> samples, int sortDir) {
	my_point p1 = points.at(i-1);
	my_point p2 = points.at(i);
	my_point newP2 = p2 + my_point{(double)dx,(double)dy};
	my_point p3 = points.at(i+1);
	my_point v12 = newP2 - p1;
	my_point v23 = p3 - newP2;
	bool shifted = false;

	if(is_valid_span(v12) && is_valid_span(v23)) {
		double distBefore = max(get_span_error(p1, p2, samples), get_span_error(p2, p3, samples));
		double distAfter = max(get_span_error(p1, newP2, samples), get_span_error(newP2, p3, samples));
		double slopeBefore = max(get_span_slope_error(p1, p2, samples), get_span_slope_error(p2, p3, samples));
		double slopeAfter = max(get_span_slope_error(p1, newP2, samples), get_span_slope_error(newP2, p3, samples));

		double orderingBefore = get_ordering_cost_WRT_segment(points, i-1, sortDir) +
			get_ordering_cost_WRT_segment(points, i, sortDir);
		double spanOneBefore = get_max_span_one_length(points);

		vector<my_point> copy = points;
		copy[i] = newP2;

		double orderingAfter = get_ordering_cost_WRT_segment(copy, i-1, sortDir) +
			get_ordering_cost_WRT_segment(copy, i, sortDir);
		double spanOneAfter = get_max_span_one_length(copy);

		//System.out.println("spanOne : " + spanOneBefore + ", " + spanOneAfter);
		//System.out.println(orderingBefore + ", " + distBefore + ", " + orderingAfter + ", " + distAfter);
		//System.out.println(orderingBefore + ", " + distBefore + ", " + sortDir);

		double costBefore = orderW*orderingBefore + distW*distBefore + spanOneW*spanOneBefore + slopeW*slopeBefore;
		double costAfter = orderW*orderingAfter + distW*distAfter + spanOneW*spanOneAfter + slopeW*slopeAfter;
		//System.out.println(jaggiesBefore + ", " + distBefore);
		//System.out.println(jaggiesAfter + "// " + distAfter);
		/*
		   double costBefore = distBefore;
		   double costAfter = distAfter;
		   */
		//System.out.println(costBefore + ", " + costAfter);
		if(costBefore -0.01 < costAfter) {

		} else {
			//System.out.println(costBefore + ", " + costAfter);
			shifted = true;
			points[i] = newP2;
			//System.out.println("SHIFT");
		}
	}
	return shifted;
}

bool try_splitting_point(vector<my_point> &points, int i, int dx, int dy,
		vector<my_point> samples, int sortDir) {
	my_point p1 = points.at(i-1);
	my_point p2 = points.at(i);
	my_point p2a = p2 + my_point{(double)dx,0};
	my_point p2b = p2 + my_point{0,(double)dy};
	my_point p3 = points.at(i+1);
	my_point v12a = p2a - p1;
	my_point v2a2b = p2b - p2a;
	my_point v2b3 = p3 - p2b;
	bool shifted = false;
	//System.out.println(v12a + ", " + v2a2b + "," + v2b3);
	if(is_valid_span(v12a) && is_valid_span(v2a2b) && is_valid_span(v2b3) &&
			v12a.x*v2a2b.x > 0 && v12a.y*v2a2b.y > 0 && v2b3.x*v2a2b.x > 0 && v2b3.y*v2a2b.y > 0) {
		double distBefore = max(get_span_error(p1, p2, samples), get_span_error(p2, p3, samples));
		double distAfter = max(
				get_span_error(p1, p2a, samples),
				max(
					get_span_error(p2a, p2b, samples),
					get_span_error(p2b, p3, samples)));
		double slopeBefore = max(get_span_slope_error(p1, p2, samples), get_span_slope_error(p2, p3, samples));
		double slopeAfter = max(
				get_span_slope_error(p1, p2a, samples),
				max(
					get_span_slope_error(p2a, p2b, samples),
					get_span_slope_error(p2b, p3, samples)));
		//System.out.println(get_span_error(p1, p2, samples) + ", " + get_span_error(p2, p3, samples));
		//System.out.println(get_span_error(p1, p2a, samples) + ", " + get_span_error(p2a, p2b, samples) + ", " + get_span_error(p2b, p3, samples));
		//System.out.println(p1 + ", " + p2a + ", " + get_span_error(p1, p2a, samples));
		//System.out.println(p2.minus(p1) + ", " + p3.minus(p2));
		//System.out.println(v12a + ", " + v2a2b + ", " + v2b3);
		double orderingBefore = get_ordering_cost_WRT_segment(points, i-1, sortDir) +
			get_ordering_cost_WRT_segment(points, i, sortDir);
		double spanOneBefore = get_max_span_one_length(points);
		vector<my_point> copy = points;
		copy[i] =  p2b;
		copy.insert(copy.begin()+i, p2a);
		double orderingAfter =	get_ordering_cost_WRT_segment(copy, i-1, sortDir) +
			get_ordering_cost_WRT_segment(copy, i, sortDir) +
			get_ordering_cost_WRT_segment(copy, i+1, sortDir);
		double spanOneAfter = get_max_span_one_length(copy);
		//orderingAfter *= 2.0/3.0;
		//System.out.println(points.get(i-1) + ", " + points.get(i) + ", " + points.get(i+1) + ", " + copy.get(i-1) + ", " + copy.get(i) + ", " + copy.get(i+1) + ", " + copy.get(i+2));
		/*
		   System.out.println(
		   get_ordering_cost_WRT_segment(points, i-1, sortDir) + ", " +
		   get_ordering_cost_WRT_segment(points, i, sortDir) + ", " +
		   get_ordering_cost_WRT_segment(copy, i-1, sortDir) + ", " +
		   get_ordering_cost_WRT_segment(copy, i, sortDir) + ", " +
		   get_ordering_cost_WRT_segment(copy, i+1, sortDir));
		   */
		//System.out.println("dist: " + distBefore + ", " + distAfter);
		//System.out.println("order: " + orderingBefore + ", " + orderingAfter);
		//jaggiesBefore /= nBefore;
		//jaggiesAfter /= nAfter;
		//System.out.println(jaggiesBefore + ", " + jaggiesAfter + ", " + sortDir);
		double costBefore = orderW*orderingBefore + distW*distBefore + spanOneW*spanOneBefore + slopeW*slopeBefore;
		double costAfter = orderW*orderingAfter + distW*distAfter + spanOneW*spanOneAfter + slopeW*slopeAfter;
		//System.out.println("cost: "+ costBefore + ", " + costAfter);

		if(costBefore-0.01 < costAfter) {}
		else {
			//System.out.println("dist: " + distBefore + ", " + distAfter);
			//System.out.println("order: " + orderingBefore + ", " + orderingAfter);
			//System.out.println(p2.minus(p1) + ", " + p3.minus(p2));
			//System.out.println(v12a + ", " + v2a2b + ", " + v2b3);
			points[i] = p2b;
			points.insert( points.begin()+i, p2a );
			shifted = true;
			//System.out.println("SPLIT");
			//System.out.println(points.get(i) + ", " + v2a2b + "," + v2b3);
		}
	}
	return shifted;
}

// merge i and i+1
bool try_merging_points(vector<my_point> &points, size_t i, int dx, int dy,
		vector<my_point> samples, int sortDir) {
	if (i+2 >= points.size()){
		return false;
	}
	my_point p1 = points.at(i-1);
	my_point p2 = points.at(i);
	my_point p3 = points.at(i+1);
	my_point p4 = points.at(i+2);
	if(abs(p2.x-p3.x) > 0.9 || abs(p2.y-p3.y) > 0.9) return false;
	my_point p23 = p2 + my_point{(double)dx, (double)dy};
	my_point v1 = p23 - p1;
	my_point v2 = p4 - p23;
	bool valid1 = is_valid_span(v1);
	bool valid2 = is_valid_span(v2);
	bool shifted = false;
	//System.out.println(v12a + ", " + v2a2b + "," + v2b3);
	if(valid1 && valid2 && v1.x*v2.x > 0 && v1.y*v2.y > 0) {
		double distBefore = max(
				get_span_error(p1, p2, samples),
				max(
						get_span_error(p2, p3, samples),
						get_span_error(p3, p4, samples)));
		double distAfter = max(get_span_error(p1, p23, samples), get_span_error(p23, p4, samples));
		double slopeBefore = max(
				get_span_slope_error(p1, p2, samples),
				max(
						get_span_slope_error(p2, p3, samples),
						get_span_slope_error(p3, p4, samples)));
		double slopeAfter = max(get_span_slope_error(p1, p23, samples), get_span_slope_error(p23, p4, samples));
		//System.out.println(p2.minus(p1) + ", " + p3.minus(p2));
		//System.out.println(v12a + ", " + v2a2b + ", " + v2b3);
		double orderingBefore =	get_ordering_cost_WRT_segment(points, i-1, sortDir) +
								get_ordering_cost_WRT_segment(points, i, sortDir) +
								get_ordering_cost_WRT_segment(points, i+1, sortDir);
		double spanOneBefore = get_max_span_one_length(points);
		vector<my_point> copy = points;
		copy.erase( copy.begin() + i+1);
		copy[i] = p23;
		double orderingAfter = get_ordering_cost_WRT_segment(copy, i-1, sortDir) +
							   get_ordering_cost_WRT_segment(points, i, sortDir);
		double spanOneAfter = get_max_span_one_length(copy);
		//System.out.println(points.get(i-1) + ", " + points.get(i) + ", " + copy.get(i-1) + ", " + copy.get(i) + ", " + copy.get(i+1));
		/*
		System.out.println(
				get_ordering_cost_WRT_segment(points, i-1, sortDir) + ", " +
				get_ordering_cost_WRT_segment(points, i, sortDir) + ", " +
				get_ordering_cost_WRT_segment(copy, i-1, sortDir) + ", " +
				get_ordering_cost_WRT_segment(copy, i, sortDir) + ", " +
				get_ordering_cost_WRT_segment(copy, i+1, sortDir));
		*/
		//System.out.println("dist: " + distBefore + ", " + distAfter);
		//System.out.println("order: " + orderingBefore + ", " + orderingAfter);
		//jaggiesBefore /= nBefore;
		//jaggiesAfter /= nAfter;
		//System.out.println(jaggiesBefore + ", " + jaggiesAfter + ", " + sortDir);
		double costBefore = orderW*orderingBefore + distW*distBefore + spanOneW*spanOneBefore + slopeW*slopeBefore;
		double costAfter = orderW*orderingAfter + distW*distAfter + spanOneW*spanOneAfter + slopeW*slopeAfter;
		if(costBefore-0.01 < costAfter) {}
		else {
			points.erase( copy.begin() + i+1);
			points[i] = p23;
			shifted = true;
			//System.out.println("MERGE");
			//System.out.println(points.get(i) + ", " + v2a2b + "," + v2b3);
		}
	}
	return shifted;
}

vector<my_point> sort_slope_spans(int sortDir, vector<my_point> points, vector<bool> sorted, Path path) {
	vector<my_point> newPoints = points;
	if(sortDir==0) {
		sorted[0] = false;
		return newPoints;
	}
	else {
		vector<my_point> samples = get_sample_points(path, 100);
		//System.out.println(path);
		int itr = 0;
		int maxItr = 30;
		bool converged = false;
		int moves = 0;
		while(true) {
			if(converged || itr >= maxItr) {
				if(itr > maxItr){
					//System.out.println(itr);
				}
				break;
			}
			for(size_t i=1; i < newPoints.size()-1; i++) {
				//System.out.println("before " + itr);
				bool shiftedW = try_shifting_point(newPoints, i, -1, 0, samples, sortDir);
				bool shiftedE = try_shifting_point(newPoints, i, +1, 0, samples, sortDir);
				bool shiftedN = try_shifting_point(newPoints, i, 0, -1, samples, sortDir);
				bool shiftedS = try_shifting_point(newPoints, i, 0, +1, samples, sortDir);
				bool converged1 = !shiftedW && !shiftedE && !shiftedN && !shiftedS;
				//System.out.println("after " + itr);

				bool shiftedNW = try_splitting_point(newPoints, i, -1, -1, samples, sortDir);
				//if(shiftedNW) i++;
				bool shiftedSW = try_splitting_point(newPoints, i, -1, +1, samples, sortDir);
				//if(shiftedSW) i++;
				bool shiftedNE = try_splitting_point(newPoints, i, +1, -1, samples, sortDir);
				//if(shiftedNE) i++;
				bool shiftedSE = try_splitting_point(newPoints, i, +1, +1, samples, sortDir);
				//if(shiftedSE) i++;
				bool converged2 = !shiftedNW && !shiftedSW && !shiftedNE && !shiftedSE;

				bool mergeW = try_merging_points(newPoints, i, -1, 0, samples, sortDir);
				bool mergeE = try_merging_points(newPoints, i, +1, 0, samples, sortDir);
				bool mergeN = try_merging_points(newPoints, i, 0, -1, samples, sortDir);
				bool mergeS = try_merging_points(newPoints, i, 0, +1, samples, sortDir);
				bool converged3 = !mergeW && !mergeE && !mergeN && !mergeS;

				converged = converged1 && converged2 && converged3;
				moves += (shiftedW ? 1 : 0) + (shiftedE ? 1 : 0) + (shiftedN ? 1 : 0) + (shiftedS ? 1 : 0) +
					(shiftedNW ? 1 : 0) + (shiftedSW ? 1 : 0) + (shiftedNE ? 1 : 0) + (shiftedSE ? 1 : 0) +
					(mergeW ? 1 : 0) + (mergeE ? 1 : 0) + (mergeN ? 1 : 0) + (mergeS ? 1 : 0);
			}
			itr ++ ;
		}
		//if(moves > 8) System.out.println("moves = " + moves);
	}
	return newPoints;
}

vector<my_point> sort_path_points(Path path, vector<my_point> pixelPath, vector<bool> sorted){
		int sortDir = get_sort_direction(path);
		//System.out.println("sortDir =" + sortDir);
		vector<my_point> sortedPixelPath = sort_slope_spans(sortDir, pixelPath, sorted, path);
		return sortedPixelPath;
}
