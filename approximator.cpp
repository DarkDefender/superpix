#include "approximator.h"

#include <vector>
#include <cassert>
#include <math.h>
#include "my_point.h"
#include "path.h"
#include "curve.h"

using namespace std;

vector<my_point> get_sample_points(Curve curve, int n) {
	assert( n > 1 );
	vector<my_point> samples;
	for(int i=0; i < n; i++) {
		double t = (double)i/(double)(n-1);
		samples.push_back(curve.get_point(t));
	}
	return samples;
}

vector<my_point> get_sample_points(Path path, int n){
	vector<double> lengths;
	double totalLength = 0;
	vector<Curve> curves = path.get_curves();
	for(size_t i=0; i < curves.size(); i++) {
		double length = curves.at(i).get_frame_len();
		lengths.push_back(length);
		totalLength += length;
	}
	vector<my_point> samples;
	for(size_t i=0; i < curves.size(); i++) {
		int samplesPerCurve = (int) round( (lengths.at(i)/totalLength*(double)n) );
		/*
		   if(samplesPerCurve<=2) {
		   System.out.println("samples: " + samplesPerCurve);
		   System.out.println("length_i = " + lengths.get(i));
		   System.out.println("totalLength = " + totalLength);
		   System.out.println("n = " + n);
		   System.out.println("curve_i = " + curves.get(i));
		   System.out.println("path = " + path);
		   }
		   */
		samplesPerCurve = max(2, samplesPerCurve);
		vector<my_point> curvePts = get_sample_points(curves.at(i), samplesPerCurve);
		if(i > 0 && !curvePts.empty()){
			curvePts.erase(curvePts.begin());
		}
		samples.insert(samples.end(), curvePts.begin(), curvePts.end());
	}
	return samples;
}

my_point get_closest_point_on_line_segment(my_point p, my_point v, my_point w) {
	// Return minimum distance between line segment vw and point p
	double L2 = length(w - v);
	L2 = L2*L2; // i.e. |w-v|^2 -  avoid a sqrt
	if (L2 == 0.0){
		return v;   // v == w case
	}
	// Consider the line extending the segment, parameterized as v + t (w - v).
	// We find projection of point p onto the line.
	// It falls where t = [(p-v) . (w-v)] / |w-v|^2
	double t = dot( (p - v), (w - v) ) / L2;
	if (t < 0.0){
		return v;       // Beyond the 'v' end of the segment
	} else if (t > 1.0){
		return w;  // Beyond the 'w' end of the segment
	}
	my_point projection = v + ((w - v)*t);  // Projection falls on the segment (v + t*(w-v))
	return projection;
}

// point to line segment
double get_distance_to_line_segment(my_point p, my_point v, my_point w) {
	my_point q = get_closest_point_on_line_segment(p, v, w);
	return distance(p, q);
}

double get_min_dist(my_point p, vector<my_point> samples){
	assert(!samples.empty());
	double minDist = -1;
	for(size_t i=0; i < samples.size()-1; i++) {
		double dist = get_distance_to_line_segment(p, samples.at(i), samples.at(i+1));
		if(minDist==-1 || dist<minDist){
			minDist = dist;
		}
	}
	return minDist;
}

int get_min_dist_ind(my_point p, vector<my_point> samples) {
	assert(!samples.empty());
	int minDistInd = -1;
	double minDist = -1;
	for(size_t i=0; i < samples.size()-1; i++) {
		double dist = get_distance_to_line_segment(p, samples.at(i), samples.at(i+1));
		if(minDistInd==-1 || dist<minDist) {
			minDistInd = i;
			minDist = dist;
		}
	}
	return minDistInd;
}
