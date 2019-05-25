#include <vector>
#include <algorithm>
#include <math.h>

#include "splitter.h"
#include "segment.h"
#include "curve.h"
#include "path.h"

using namespace std;

void sort_and_remove_duplicates(vector<double> &vec){
	//Sort vector
	sort(vec.begin(), vec.end());
	//Erase duplicates
	vec.erase( unique( vec.begin(), vec.end() ), vec.end() );
}

bool getSegmentFlag(Path path, int i) {
	bool flag = false;
	// is segment i nondifferentiable
	if(path.is_nondifferentiable_at_segment_index(i)) {
		flag = true;
	} else {
		// is segment i a local minimum
		Curve curveBefore = path.get_curve_before_seg_idx(i);
		Curve curveAfter = path.get_curve_after_seg_idx(i);
		if (curveBefore.is_max_at1('x') && curveAfter.is_max_at0('x')){
			flag = true;
		} else if (curveBefore.is_min_at1('x') && curveAfter.is_min_at0('x')){
			flag = true;
		} else if (curveBefore.is_max_at1('y') && curveAfter.is_max_at0('y')){
			flag = true;
		} else if (curveBefore.is_min_at1('y') && curveAfter.is_min_at0('y')){
			flag = true;
		} else {
			flag = false;
		}
	}
	return flag;
}

// returns a vector of segments describing the split curve
// flag = true means a point is a critical point
// endpoints have flag = false for now, unless they are critical points
// the status of endpoints can change later depending on the original
// curve's position in the path
vector<FlaggedSegment> split_curve_by_monotonicity(Curve curve0, bool with_min_max, bool with_inflection_pts){
	vector<double> splitTs;
	if(with_min_max) {
		vector<double> xCritPts = curve0.t_values_at_local_min_max('x');
		vector<double> yCritPts = curve0.t_values_at_local_min_max('y');
		//Add all crit pts
		splitTs.insert(splitTs.end(), xCritPts.begin(), xCritPts.end());
		splitTs.insert(splitTs.end(), yCritPts.begin(), yCritPts.end());
	}
	if(with_inflection_pts) {
		vector<double> inflectPts = curve0.t_values_at_inflection_points();
		splitTs.insert(splitTs.end(), inflectPts.begin(), inflectPts.end());
	}
	sort_and_remove_duplicates(splitTs);

	// remove 0's and 1's. they will be tested later
	if(!splitTs.empty() && splitTs.at(0) == 0){
		splitTs.erase(splitTs.begin());
	}

	if(!splitTs.empty() && splitTs.at(splitTs.size() - 1) == 1){
		splitTs.pop_back();
	}

	vector<double> ts;
	ts.insert(ts.end(), splitTs.begin(), splitTs.end());
	ts.push_back(0.0);
	ts.push_back(1.0);
	sort_and_remove_duplicates(ts);

	//cout << "ts: " << ts << endl;

	vector<Curve> splitCurves;
	vector<bool> flags;
	for(size_t j=0; j < ts.size()-1; j++) {
		Curve curve = curve0.get_sub_curve(ts.at(j), ts.at(j+1));
		splitCurves.push_back(curve);
		if(j==0) {
			flags.push_back(!splitTs.empty() && splitTs.at(0) == 0);
		}
		else {
			flags.push_back(!splitTs.empty());
			if(j==ts.size()-2) {
				flags.push_back(!splitTs.empty() && splitTs.at(splitTs.size()-1) == 1);
			}
		}
	}
	vector<FlaggedSegment> splitSegments;
	for(size_t k=0; k < splitCurves.size(); k++) {
		Curve curve = splitCurves.at(k);
		FlaggedSegment seg1, seg2;
		if(k == 0) {
			my_point point = curve.get_pN(0), handleOut = curve.get_handle1();
			seg1 = FlaggedSegment{point, handleOut * (-1), handleOut};
			seg1.set_flag(flags.at(k));
			if ( curve0.is_straight() ){
				seg1.set_flag(true);
			}
		} else {
			Curve prevCurve = splitCurves.at(k-1);
			my_point point = curve.get_pN(0), handleIn = prevCurve.get_handle2(), handleOut = curve.get_handle1();
			seg1 = FlaggedSegment{point, handleIn, handleOut};
			seg1.set_flag(flags.at(k));
			if(prevCurve.is_straight() || curve.is_straight()){
				seg1.set_flag(true);
			}
		}
		splitSegments.push_back(seg1);
		if( k == splitCurves.size() - 1 ) {
			my_point point = curve.get_pN(3), handleIn = curve.get_handle2();
			seg2 = FlaggedSegment{point, handleIn, handleIn * (-1)};
			if(splitTs.empty()){
				seg2.set_flag(false);
			} else if (splitTs.at(splitTs.size()-1) < 1){
				seg2.set_flag(false);
			} else {
				seg2.set_flag(true);
			}
			if(curve0.is_straight()) {
				seg2.set_flag(true);
			}
			splitSegments.push_back(seg2);
		}
	}

	return splitSegments;
}

// locate the min/max points and add them as control points
vector<FlaggedSegment> get_segments_split_at_critical_points(Path path, bool minMax, bool inflection, bool straightLineEnds) {
	vector<FlaggedSegment> newPathSegments;

	for(size_t i=0; i < path.get_num_curves(); i++) {
		// flags indicating whether a segment is at a split point
		vector<FlaggedSegment> splitSegments = split_curve_by_monotonicity(path.get_curve(i), minMax, inflection);
		size_t n = splitSegments.size();
		FlaggedSegment *firstSeg = &splitSegments.at(0);
		FlaggedSegment *lastSeg = &splitSegments.at(n-1);
		//bool firstFlag = getSegmentFlag(path, i);
		//System.out.println(path.getCurve(i).is_straight() + ", " + path.getCurve(i+1).is_straight());
		bool firstFlag = getSegmentFlag(path, i);// || i==0 || MyMath.xor(path.getCurve(i-1).is_straight(), path.getCurve(i).is_straight());
		//System.out.println(i + " : " + firstFlag + ", " + firstSeg);
		bool lastFlag = getSegmentFlag(path, i+1);
		if ( i == path.get_num_curves()-1 && !splitSegments.empty() && path.is_closed()) {
			lastFlag = splitSegments.at(0).get_flag();
		}
		if(straightLineEnds) {
			if(path.get_curve(i).is_straight() || (i>0 && path.get_curve(i-1).is_straight())) {
				firstFlag = true;
			}
			if(path.get_curve(i).is_straight() || (i < n-1 && path.get_curve(i+1).is_straight())) {
				lastFlag = true;
			}
		}

		firstSeg->set_flag(firstFlag);
		lastSeg->set_flag(lastFlag);
		if(!newPathSegments.empty()) {
			FlaggedSegment *lastSegAdded = &newPathSegments.at(newPathSegments.size()-1);
			firstSeg->set_handle_in(lastSegAdded->get_handle_in());

			if(straightLineEnds && path.get_curve(i).is_straight()) {
				firstSeg->set_flag(true);
				lastSegAdded->set_flag(true);
			}

			newPathSegments.pop_back();
		}

		newPathSegments.insert(newPathSegments.end(), splitSegments.begin(), splitSegments.end());
		/*
		if(path.getCurve(i).is_straight() & splitSegments.size()>=2) {
			System.out.println(firstSeg + " ... " + lastSeg);
		}
		*/
	}
	return newPathSegments;
}

// split into different paths at the min/max points (now control points)
vector<Path> merge_flagged_segments(vector<FlaggedSegment> segments) {
	vector<Path> newPaths;
	Path newPath;
	for(size_t i=0; i < segments.size(); i++) {
		FlaggedSegment segment = segments.at(i);
		//System.out.println("flag " + i + " : " + segment);
		/*
		if(1+1==2) {
		newPath.add_segment(segment.clone());
		newPaths.add(newPath.clone());
		newPath.clear();
		newPath.add_segment(segment.clone());
		continue;
		}*/
		if(i==0) {
			newPath.clear();
			newPath.add_segment(segment);
		} else if (segment.get_flag()) {
			newPath.add_segment(segment);
			newPaths.push_back(newPath);
			newPath.clear();
			newPath.add_segment(segment);
		} else if (i == segments.size()-1) {
			newPath.add_segment(segment);
			newPaths.push_back(newPath);
		} else {
			newPath.add_segment(segment);
		}
	}
	my_point p1 = segments.at(0).get_point();
	my_point p2 = segments.at(segments.size()-1).get_point();
	bool closed = abs(p1.x-p2.x) < 0.001 && abs(p1.y-p2.y) < 0.001;
	if(closed && segments.size() >= 2) {
		FlaggedSegment segment0 = segments.at(0);
		if(!segment0.get_flag()) {
			Path firstPath = newPaths.at(0);
			Path lastPath = newPaths.at(newPaths.size()-1);
			for(size_t i=0; i < firstPath.get_num_curves(); i++) {
				lastPath.add_curve(firstPath.get_curve(i));
			}
			newPaths.erase(newPaths.begin());
		}
	}
	return newPaths;
}

vector<Path> split_by_monotonicity(Path path, bool min_max, bool inflection, bool straight_line_ends) {
	//cout << path;
	//cout << "path size: " << path.get_num_segments() << endl;
	vector<FlaggedSegment> unsplitPathSegments = get_segments_split_at_critical_points(path, min_max, inflection, straight_line_ends);
	//cout << unsplitPathSegments;
	//cout << "before: " << unsplitPathSegments.size() << endl;
	vector<Path> splitPaths = merge_flagged_segments(unsplitPathSegments);
	//cout << "after: " << splitPaths.size() << endl;
	//cout << splitPaths;
	return splitPaths;
}

vector<Path> split_by_monotonicity(Path path) {
	return split_by_monotonicity(path, true, true, false);
}

