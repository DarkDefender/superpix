#include "path.h"

#include <math.h>
#include <algorithm>
#include <cassert>
#include <iomanip>

Path::Path(){

}

Path::Path(my_point p1, my_point p2){
	segments.push_back(Segment{p1});
	segments.push_back(Segment{p2});
}

Path::Path(vector<Segment> segments){
	this->segments = segments;
}

bool Path::empty(){
	return segments.empty();
}

void Path::clear(){
	segments.clear();
}

Path Path::get_transformed_copy(Transform<double,2,Affine> at) {
	Path newPath = Path{segments};
	newPath.transform(at);
	return newPath;
}

void Path::transform(Transform<double,2,Affine> at) {
	for (auto& seg : segments) {
		seg.transform(at);
	}
}

size_t Path::get_num_segments() const{
	return segments.size();
}

size_t Path::get_num_curves() const{
	return max(0, (int)segments.size()-1);
}

Path Path::get_reverse_path(){
	vector<Segment> rev_vec = segments;
	reverse(rev_vec.begin(), rev_vec.end());
	return Path{rev_vec};
}

Curve Path::get_curve(int i) const{
	// Is the curve index valid?
	assert(i >= 0 && i < get_num_curves());

	my_point p0 = segments.at(i).get_point();
	my_point p1 = p0 + segments.at(i).get_handle_out();
	my_point p3 = segments.at(i+1).get_point();
	my_point p2 = p3 + segments.at(i+1).get_handle_in();
	if(isnan(p0.x) || isnan(p0.y)) {
		//System.out.println("getCurve(" + i + ")");
	}
	return Curve{p0,p1,p2,p3};
}

bool Path::is_valid_seg_idx(int i){
	return (i==0 && !is_closed());
}

Curve Path::get_curve_before_seg_idx(int i){
	if (is_valid_seg_idx(i)){
		//TODO see if we have to fix up other stuff now that we do not return null
		//return null;
		assert(false);
	}
	int curveInd = (i==0) ? segments.size()-2 : i-1;
	Curve curveBefore = get_curve(curveInd);
	return curveBefore;
}

Curve Path::get_curve_after_seg_idx(int i){
	if(is_valid_seg_idx(i)){
		//TODO see if we have to fix up other stuff now that we do not return null
		//return null;
		assert(false);
	}
	int curveInd = (i==segments.size()-1) ? 0 : i;
	Curve curveBefore = get_curve(curveInd);
	return curveBefore;
}

vector<Curve> Path::get_curves() const{
	vector<Curve> myCurves;
	for(size_t i=0; i < get_num_curves(); i++) {
		myCurves.push_back(get_curve(i));
	}
	return myCurves;
}

Segment Path::get_segment(int i) const{
	return segments.at(i);
}

Segment Path::get_first_segment(){
	return segments.front();
}

Segment Path::get_last_segment(){
	return segments.back();
}

my_point Path::get_first_point(){
	return segments.front().get_point();
}

my_point Path::get_last_point(){
	return segments.back().get_point();
}

my_point Path::get_tangent_in_at_segment_index(int i){
		if(!is_valid_seg_idx(i)){
			return my_point{0,0};
		}
		Curve curveBefore = get_curve_before_seg_idx(i);
		my_point p0 = curveBefore.get_pN(0), p1 = curveBefore.get_pN(1), p2 = curveBefore.get_pN(2);
		my_point p = segments.at(i).get_point();
		my_point v0 = p0 - p, v1 = p1 - p, v2 = p2 - p;
		if(length(v2) > 0.001){
			return v2;
		} else if(length(v1) > 0.001){
			return v1;
		}
		return v0;
}

my_point Path::get_tangent_out_at_segment_index(int i){
		if(!is_valid_seg_idx(i)){
			return my_point{0,0};
		}
		Curve curveAfter = get_curve_after_seg_idx(i);
		my_point p1 = curveAfter.get_pN(1), p2 = curveAfter.get_pN(2), p3 = curveAfter.get_pN(3);
		my_point p = segments.at(i).get_point();
		my_point v1 = p1 - p, v2 = p2 - p, v3 = p3 - p;
		if(length(v1) > 0.001){
			return v1;
		} else if(length(v2) > 0.001){
			return v2;
		}
		return v3;
}

void Path::set(int i, Segment newSegment) {
	if(is_closed()) {
		int n = get_num_segments()-1;
		if(i==0 || i==n) {
			segments[0] = newSegment;
			segments[n] = newSegment;
		} else {
			segments[i] = newSegment;
		}
	} else {
		segments[i] = newSegment;
	}
	//if(i==1) System.out.println(newSegment);
}

void Path::set_last_segment(Segment seg){
	set(get_num_segments()-1, seg);
}

// output = {topleft, bottomright}
vector<my_point> Path::get_bounding_box() {
	assert(get_num_segments() >= 1);
	if(get_num_segments() == 0) {
		my_point p = get_segment(0).get_point();
		vector<my_point> bb{p, p};
		return bb;
	} else {
		vector<my_point> bb = get_curve(0).get_bounding_box();
		my_point p1 = bb[0], p2 = bb[1];
		for(size_t i=0; i < get_num_curves(); i++) {
			Curve curve = get_curve(i);
			vector<my_point> curveBB = curve.get_bounding_box();
			// xmin
			if(curveBB[0].x < p1.x) p1.x = curveBB[0].x;
			// ymin
			if(curveBB[0].y < p1.y) p1.y = curveBB[0].y;
			// xmax
			if(curveBB[1].x > p2.x) p2.x = curveBB[1].x;
			// ymax
			if(curveBB[1].y > p2.y) p2.y = curveBB[1].y;
		}
		vector<my_point> newBB{p1, p2};
		return newBB;
	}
}

bool Path::is_acute_angle(int i) {
	int n = get_num_segments();
	assert(0 <= i && i < n);
	if((i==0 || i==n-1) && !is_closed()){
		return false;
	} else {
		Curve c1 = get_curve_before_seg_idx(i);
		Curve c2 = get_curve_after_seg_idx(i);
		my_point tan1 = c1.get_tangent(0.99) * (-1);
		my_point tan2 = c2.get_tangent(0.01);
		//System.out.println(c1+ ", " + c2);
		double angleTo = get_angle_to(tan1, tan2);
		bool acute = abs(angleTo) <= 0.5*M_PI;
		//if(acute) System.out.println(tan1 + ", " + tan2 + ", " + angleTo);
		return acute;
	}
}

bool Path::is_closed() {
	my_point p1 = get_first_point();
	my_point p2 = get_last_point();
	return (abs(p1.x-p2.x) < 0.001) && (abs(p1.y-p2.y) < 0.001);
}

bool Path::is_straight() {
	double frameLength = 0;
	vector<Curve> curves = get_curves();
	for(size_t i=0; i < curves.size(); i++) {
		frameLength += curves.at(i).get_frame_len();
		//segments.get(i).getPoint().distance(segments.get(i-1).getPoint());
		//System.out.println(i + " : " + frameLength);
	}
	double straightDist = distance(get_first_point(), get_last_point());
	//cout << *this;
	//cout << frameLength << ", " << straightDist << endl;
	return abs(straightDist-frameLength) < 0.001;
}

// assumes no degenerate curves in the path
bool Path::is_nondifferentiable_at_segment_index(int i) {
	//Valid segment index?
	assert(i >= 0 && i< segments.size());
	if(!is_closed() && (i==0 || i==segments.size()-1)){
		return true;
	} else {
		my_point tangentIn = get_tangent_in_at_segment_index(i);
		my_point tangentOut = get_tangent_out_at_segment_index(i);
		double x1 = tangentIn.x, y1 = tangentIn.y, x2 = tangentOut.x, y2 = tangentOut.y;
		bool smooth =  abs(x1*y2-x2*y1) < 0.001;
		return !smooth;
	}
}

void Path::add_segment(Segment seg){
	segments.push_back(seg);
}

void Path::add_curve(Curve c){
	if( empty() ) {
		add_segment( Segment{c.get_pN(0), c.get_handle1()*(-1), c.get_handle1()} );
	} else {
		Segment lastSeg = get_last_segment();
		lastSeg.set_handle_out(c.get_handle1());
		set_last_segment(lastSeg);
	}
	add_segment( Segment{c.get_pN(3), c.get_handle2(), c.get_handle2()*(-1)} );
}

void Path::remove_segment(int i) {
	//Valid segment index?
	assert(i >= 0 && i< segments.size());
	auto it = segments.begin();
	if(is_closed()) {
		int n = get_num_segments()-1;
		if(i==0 || i==n) {
			segments.erase(it+n);
			segments.erase(it+0);
		}
		else {
			segments.erase(it+i);
		}
	}
	else {
		segments.erase(it+i);
	}
}

ostream& operator <<(ostream &o, const Path &path){
	o << "Path:" << endl;
	//Set number of printed decimals to 2
	o << fixed << setprecision(2);
	for( size_t i = 0; i < path.get_num_segments(); i++){
		o << "  ";
		o << path.get_segment(i);
		o << endl;
	}
	o << endl;

	vector<Curve> curves = path.get_curves();
	for(auto curve : curves){
		o << "  Curve: ";
		o << "p0: "<< curve.get_pN(0) << ", ";
		o << "p1: "<< curve.get_pN(1) << ", ";
		o << "p2: "<< curve.get_pN(2) << ", ";
		o << "p3: "<< curve.get_pN(3) << ", ";
		o << endl;
	}

	return o;
}

ostream& operator <<(ostream &o, const vector<Path> &paths){
	for (size_t i = 0; i < paths.size(); i++){
		o << "Path No: " << i << endl;
		o << paths[i];
	}
	return o;
}
