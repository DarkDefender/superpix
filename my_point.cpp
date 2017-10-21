#include "my_point.h"
#include <math.h>

bool in_range(double val, double a, double b){
	return (b-val)*(val-a) >= 0;
}

double distance(const my_point &a, const my_point &b){
	double len = sqrt( pow(b.x - a.x, 2) + pow(b.y - a.y, 2));
    return len;
}

double length(const my_point &a){
	my_point zero{0,0};
	return distance(zero, a);
}

double dot(const my_point &a, const my_point &b){
	double dot_prod = a.x*b.x + a.y*b.y;
	return dot_prod;
}

// get angle of vector; range: [0, 2pi)
// vector has start point in the origin
double get_angle(const my_point &a) {
	if (is_zero(a)){
		// cannot both be zero vector
		return 0;
	}
	// y is reversed
	double yy = -a.y; // reversed y
	if(a.x == 0){
		return (yy > 0) ? 0.5*M_PI : 1.5*M_PI;
	} else if(yy == 0) {
		return (a.x > 0) ? 0 : M_PI;
	} else {
		double absAngle = abs(atan(yy/a.x)); // (0, pi/2)
		if(a.x > 0 && yy > 0){
			// Quadrant I
			return absAngle;
		} else if(a.x < 0 && yy > 0){
			// Quadrant II
			return M_PI - absAngle;
		} else if(a.x < 0 && yy < 0){
			// Quadrant III
			return absAngle + M_PI;
		} else {
			// Quadrant IV
			return 2*M_PI - absAngle;
		}
	}
}


// get smallest angle to another vector
double get_angle_to(const my_point &a, const my_point &b) {
	double angle1 = get_angle(a);
	double angle2 = get_angle(b);
	if(angle2 < angle1) angle2 += 2*M_PI;
	double angleTo = angle2 - angle1;
	if(angleTo > M_PI) {
		angleTo = -(2*M_PI-angleTo);
	}
	if(abs(abs(angleTo) -0.5*M_PI) < 0.01){
		angleTo = 0.5*M_PI;
	}
	return angleTo;
}

bool is_zero(const my_point &p){
	if(p.x == 0 && p.y == 0){
		return true;
	}
	return false;
}

bool operator ==(const my_point &a, const my_point &b){
	return (a.x == b.x) && (a.y == b.y);
}

my_point operator -(const my_point &a, const my_point &b){
	my_point new_p;

	new_p.x = a.x - b.x;
	new_p.y = a.y - b.y;

	return new_p;
}

my_point operator +(const my_point &a, const my_point &b){
	my_point new_p;

	new_p.x = a.x + b.x;
	new_p.y = a.y + b.y;

	return new_p;
}

my_point operator *(const my_point &a, const double &b){
	my_point new_p;

	new_p.x = a.x * b;
	new_p.y = a.y * b;

	return new_p;
}

ostream& operator <<(ostream &o, const my_point &p){
	o << "(" << p.x << ", " << p.y << ")";
	return o;
}

ostream& operator <<(ostream &o, vector<double> vec_d){
	o << "[";
	for (size_t i = 0; i < vec_d.size() -1; i++){
		o << vec_d[i] << ", ";
	}
	o << vec_d.back() << "]" << endl;
	return o;
}
