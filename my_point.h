#ifndef MY_POINT
#define MY_POINT

#include <iostream>
#include <vector>

using namespace std;

//TODO perhaps replace this with eigen3 vectors instead
struct my_point {
	//TODO use floats instead?
	double x;
	double y;
};

bool in_range(double val, double a, double b);

double distance(const my_point &a, const my_point &b);

double length(const my_point &a);

double dot(const my_point &a, const my_point &b);

//Get angle from the origin
double get_angle(const my_point &a);

double get_angle_to(const my_point &a, const my_point &b);

bool is_zero(const my_point &p);

bool operator ==(const my_point &a, const my_point &b);
my_point operator -(const my_point &a, const my_point &b);
my_point operator +(const my_point &a, const my_point &b);
my_point operator *(const my_point &a, const double &b);

ostream& operator <<(ostream &o, const my_point &p);

ostream& operator <<(ostream &o, vector<double> vec_d);

#endif
