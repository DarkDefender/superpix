#include "polygonalizer.h"

#include <cassert>
#include <vector>
#include <math.h>

#include <iostream>

#include "path.h"
#include "my_point.h"
#include "approximator.h"
#include "sorter.h"
#include "sorter2.h"

using namespace std;

vector<my_point> reverse_vector(vector<my_point> points) {
	vector<my_point> reverse;
	for(int i = points.size()-1; i >= 0; i--) {
		reverse.push_back(points.at(i));
	}
	return reverse;
}

bool is_half(double v) {
	return abs( abs(fmod(v,1)) - 0.5 ) < 0.01;
}


int get_best_candidate(vector<my_point> candidates, my_point currPt, Path path, vector<my_point> samples,
		double posW, double slopeW, my_point centre) {
	assert(posW>=0 && slopeW>=0 && posW+slopeW>0);
	// calculate the error associated with each candidate
	vector<double> errors;

	for(size_t i=0; i < candidates.size(); i++) {
		my_point nextPt = candidates.at(i);
		double distErr = get_min_dist(nextPt, samples);
		/*
		my_point closestSlope1 = getApproxSlopeOfClosestPt(currPt, samples);
		my_point closestSlope2 = getApproxSlopeOfClosestPt(nextPt, samples);
		my_point closestSlope = (closestSlope1.plus(closestSlope2)).times(0.5);
		my_point corner1 = (currPt.x < nextPt.x) ? currPt.minus(new my_point(0.5,0.5))
												: currPt.plus(new my_point(0.5,0.5));
		my_point corner2 = (currPt.x < nextPt.x) ? nextPt.minus(new my_point(0.5,0.5))
												: nextPt.plus(new my_point(0.5,0.5));
		my_point slope = corner2.minus(corner1);
		double slopeErr = abs(closestSlope.getAngleTo(slope));
		double error = posW*distErr + slopeW*slopeErr;
		*/
		double error = distErr;
		errors.push_back(error);
	}
	double minError = *min_element(errors.begin(), errors.end());
	vector<int> minInds;
	for(size_t i=0; i < candidates.size(); i++) {
		//System.out.println(i + " : " + errors.get(i));
		if(abs(errors.at(i) - minError) < 0.001) {
			minInds.push_back(i);
		}
	}
	if(minInds.empty()) {
		for(size_t i=0; i < candidates.size(); i++) {
			//System.out.println(i + " : " + candidates.get(i) + ", " + currPt + ", " + samples.size() + ", " + errors.get(i) + ", " + (path==null));
		}
	}
	assert(!minInds.empty());
	// get the one closest to the centre
	int minInd = minInds.at(0);
	my_point bestPt = candidates.at(minInd);
	for(size_t i=1; i < minInds.size(); i++) {
		my_point bestPt2 = candidates.at(minInds.at(i));
		if(distance(bestPt2, centre) < distance(bestPt, centre)) {
			minInd = minInds.at(i);
			bestPt = candidates.at(minInd);
		}
	}
	return minInd;
}


// assume canonical form
vector<my_point> convert_pixel_centres_to_corner_path(vector<my_point> centres) {
	vector<my_point> corners;
	if(centres.empty()){
		return corners;
	} else {
		my_point p = centres.at(0);
		corners.push_back( my_point{floor(p.x), floor(p.y)});
		for(size_t i=0; i < centres.size(); i++) {
			my_point next = centres.at(i);
			if(abs(next.x-p.x) > 0.5 && abs(next.y-p.y) > 0.5) {
				p = next;
				corners.push_back( my_point{floor(p.x), floor(p.y)});
			}

			if(i==centres.size()-1) {
				my_point last = centres.at(i);
				corners.push_back( my_point{ceil(last.x), ceil(last.y)});
			}

		}
		return corners;
	}
}


bool is_extra_pixel(vector<my_point> points, size_t i) {
	if( i <= 0 || i >= points.size()-1){
		return false;
	} else {
		my_point q1 = points.at(i-1);
		my_point q2 = points.at(i);
		my_point q3 = points.at(i+1);
		return ( (q2.x-q1.x) == 0 && (q3.y-q2.y) == 0) || ((q2.y-q1.y) == 0 && (q3.x-q2.x) == 0);
	}
}

void remove_extra_pixels(vector<my_point> &points, vector<my_point> samples) {
	for(int i=points.size()-2; i>=1; i--) {
		//my_point q1 = points.get(i-1);
		//my_point q2 = points.get(i);
		//my_point q3 = points.get(i+1);
		if(is_extra_pixel(points, i)) {
			my_point q1 = points.at(i-1);
			my_point q2 = points.at(i);
			my_point q3 = points.at(i+1);
			bool remove = true;
			double d1 = get_min_dist(q1, samples);
			double d2 = get_min_dist(q2, samples);
			double d3 = get_min_dist(q3, samples);
			if(is_extra_pixel(points, i-1) && d1 < d2){
				remove = false;
			} else if(is_extra_pixel(points, i+1) && d3 < d2){
				remove = false;
			}
			if(remove) {
				points.erase(points.begin()+i);
				i--;
			}
		}
	}
}

// assume increasing value and increasing positive slope
vector<my_point> plot_canonical_unsorted_path_bresenham_WRT_center(Path path, my_point firstPt, my_point lastPt, my_point centre) {
	vector<my_point> points;
	double startX = is_half(fmod(firstPt.x,1)) ? round(firstPt.x-0.5)+0.5 : round(firstPt.x) + 0.5;
	double startY = is_half(fmod(firstPt.y,1)) ? round(firstPt.y-0.5)+0.5 : round(firstPt.y) + 0.5;
	double endX = is_half(fmod(lastPt.x,1)) ? round(lastPt.x-0.5)+0.5 : round(lastPt.x) - 0.5;
	double endY = is_half(fmod(lastPt.y,1)) ? round(lastPt.y-0.5)+0.5 : round(lastPt.y) - 0.5;
	//System.out.println(lastPt.x + ", " + centre.x + ", " + Myceil((int)lastPt.x,centre.x));
	//System.out.println(lastPt + "~~" + endX + "~~ " + endY);
	/*
	   double
	   endX = ceil(lastPt.x) - 0.5,
	   endY = ceil(lastPt.y) - 0.5;*/
	vector<my_point> points1;
	vector<my_point> points2;
	my_point p1 = my_point{startX, startY};
	my_point p2 = my_point{endX, endY};
	//System.out.print(firstPt + ", " + lastPt + " --> ");
	//System.out.println(p1 + ", " + p2);
	if(path.is_straight()) {
		path = Path{p1, p2};
	}
	vector<my_point> samples = get_sample_points(path, 100);
	//if(samples.isEmpty()) System.out.println(path);
	points1.push_back(p1);
	points2.insert(points2.begin(),p2);
	while(abs(p1.x-p2.x)>0 && abs(p1.y-p2.y)>0) {
		// for path 1
		{
			vector<my_point> candidates;
			my_point q1 = p1 + my_point{1,0};
			my_point q2 = p1 + my_point{0,1};
			my_point q3 = p1 + my_point{1,1};
			if(in_range(q1.x, p1.x, p2.x) && in_range(q1.y, p1.y, p2.y)) {
				candidates.push_back(q1);
			}
			if(in_range(q2.x, p1.x, p2.x) && in_range(q2.y, p1.y, p2.y)) {
				candidates.push_back(q2);
			}
			if(in_range(q3.x, p1.x, p2.x) && in_range(q3.y, p1.y, p2.y)) {
				candidates.push_back(q3);
			}
			if(candidates.empty()) {
				break;
			} else {
				my_point spanStart;
				for(int j = points1.size()-1; j >= 0; j--) {
					my_point p = points1.at(j);
					if(p.x==p1.x || p.y==p1.y){
						spanStart = p;
					} else {
						break;
					}
				}
				int bestInd = get_best_candidate(candidates, spanStart, path, samples, 1, 0.1, centre);
				p1 = candidates.at(bestInd);
				points1.push_back(p1);
			}
		}
		// for path 2
		{
			vector<my_point> candidates;
			my_point q1 = p2 + my_point{-1,0};
			my_point q2 = p2 + my_point{0,-1};
			my_point q3 = p2 + my_point{-1,-1};
			if(in_range(q1.x, p1.x, p2.x) && in_range(q1.y, p1.y, p2.y)) {
				candidates.push_back(q1);
			}
			if(in_range(q2.x, p1.x, p2.x) && in_range(q2.y, p1.y, p2.y)) {
				candidates.push_back(q2);
			}
			if(in_range(q3.x, p1.x, p2.x) && in_range(q3.y, p1.y, p2.y)) {
				candidates.push_back(q3);
			}
			if(candidates.empty()) {
				break;
			}
			else {
				my_point spanStart;
				for(size_t j=0; j <= points2.size()-1; j++) {
					my_point p = points2.at(j);
					if(p.x == p2.x || p.y == p2.y){
						spanStart = p;
					} else {
						break;
					}
				}
				int bestInd = get_best_candidate(candidates, spanStart, path, samples, 1, 0.1, centre);
				//p2 = candidates.get(bestInd).clone();
				//points2.add(0,p2.clone());
			}
		}
	}

	points.insert(points.end(), points1.begin(), points1.end());
	points.insert(points.end(), points2.begin(), points2.end());
	// remove L-shaped corners
	remove_extra_pixels(points, samples);
	vector<my_point> corners = convert_pixel_centres_to_corner_path(points);
	//for(int i=0; i<points.size(); i++) System.out.print(points.get(i) + ", ");
	//System.out.println();
	return corners;
}

vector<my_point> plot_unsorted_path_WRT_center(Path path, my_point centre) {
	my_point firstPt = path.get_first_point();
	my_point lastPt = path.get_last_point();
	my_point c = centre;
	double dirX = (firstPt.x <= lastPt.x) ? 1 : -1;
	double dirY = (firstPt.y <= lastPt.y) ? 1 : -1;
	my_point p1 = firstPt;
	my_point p2 = lastPt;
	Path newPath1;
	// orient
	for(size_t i=0; i<path.get_num_segments(); i++) {
		Segment s = path.get_segment(i);
		my_point p = my_point{s.get_point().x*dirX, s.get_point().y*dirY};
		my_point hi = my_point{s.get_handle_in().x*dirX, s.get_handle_in().y*dirY};
		my_point ho = my_point{s.get_handle_out().x*dirX, s.get_handle_out().y*dirY};
		newPath1.add_segment(Segment{p, hi, ho});
	}
	p1 = my_point{p1.x*dirX, p1.y*dirY};
	p2 = my_point{p2.x*dirX, p2.y*dirY};
	c = my_point{c.x*dirX, c.y*dirY};
	bool flipXY = get_sort_direction(newPath1) < 0;
	Path newPath2;
	// flip x and y
	if(flipXY) {
		for(size_t i=0; i < newPath1.get_num_segments(); i++) {
			Segment s = newPath1.get_segment(i);
			my_point p  = my_point{s.get_point().y, s.get_point().x};
			my_point hi = my_point{s.get_handle_in().y, s.get_handle_in().x};
			my_point ho = my_point{s.get_handle_out().y, s.get_handle_out().x};
			newPath2.add_segment(Segment{p, hi, ho});
		}
		p1 = my_point{p1.y, p1.x};
		p2 = my_point{p2.y, p2.x};
		c  = my_point{c.y, c.x};
	} else {
		newPath2 = newPath1;
	}
	// pixelate
	vector<my_point> pixelPath = plot_canonical_unsorted_path_bresenham_WRT_center(newPath2, p1, p2, c);
	/*
	   if(abs(lastPt.x-centre.x) < 1 && abs(lastPt.y-centre.y) < 1) {
	   System.out.println(firstPt + ", " + lastPt + ", " + centre);
	   System.out.println(p1 + "... " + p2 + "... " + c);
	   System.out.println(pixelPath.get(0) + "// " + pixelPath.get(pixelPath.size()-1));
	   }*/
	// unflip
	if(flipXY) {
		for(size_t i=0; i < pixelPath.size(); i++) {
			my_point p = pixelPath.at(i);
			pixelPath[i] = my_point{p.y, p.x};
		}
	}
	// reorient
	for(size_t i=0; i < pixelPath.size(); i++) {
		my_point p = pixelPath.at(i);
		pixelPath[i] = my_point{p.x*dirX, p.y*dirY};
	}
	return pixelPath;
}

bool first_point_closer_to_centre_than_last_point(Path path, my_point centre) {
	if(path.get_num_segments() < 2){
		return true;
	} else {
		my_point p1 = path.get_first_point();
		my_point p2 = path.get_last_point();
		double dx1 = abs(p1.x - centre.x);
		double dx2 = abs(p2.x - centre.x);

		if(dx1 < dx2 - 0.1){
			return true;
		}
		if(dx2 < dx1 - 0.1){
			return false;
		}

		double dy1 = abs(p1.y - centre.y);
		double dy2 = abs(p2.y - centre.y);
		if(dy1 < dy2){
			return true;
		}
		return false;
	}
}

vector<my_point> get_pixelated_path_WRT_center(Path path, bool toSort, my_point centre) {
	toSort = true;

	vector<my_point> pixelPath;
	bool reverse = !first_point_closer_to_centre_than_last_point(path, centre);
	pixelPath = plot_unsorted_path_WRT_center(reverse ? path.get_reverse_path() : path, centre);

	if(toSort) {
		vector<bool> sorted = {true};
		vector<my_point> sortedPixelPath = sort_path_points(
				reverse ? path.get_reverse_path() : path, pixelPath, sorted);
		if(reverse){
			sortedPixelPath = reverse_vector(sortedPixelPath);
		}
		if(reverse){
			pixelPath = reverse_vector(pixelPath);
		}
		// fix kinks from slope +-1 segments
		if(sorted[0]) {
			return sortedPixelPath;
		} else {
			return pixelPath;
		}
	} else {
		if(reverse){
			pixelPath = reverse_vector(pixelPath);
		}
		return pixelPath;
	}
}

vector<my_point> get_pixelated_path_WRT_center(vector<Path> paths, bool sorted, my_point center){
		vector<my_point> shapePixels;
		for(size_t i=0; i < paths.size(); i++) {
			Path path = paths.at(i);
			// if length 0, don't do anything
			if(path.get_first_point() == path.get_last_point()){
				//cout << "Got lenght 0 in polygonalizer\n";
				continue;
			}
			//cout << i << endl;
			//cout << path;
			vector<my_point> new_pix = get_pixelated_path_WRT_center(paths.at(i), sorted, center);
			//cout << new_pix;

			shapePixels.insert(shapePixels.end(), new_pix.begin(), new_pix.end());
		}
		return shapePixels;
}
