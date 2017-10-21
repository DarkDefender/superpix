#include "curve.h"

#include <math.h>
#include <cassert>

#include "my_point.h"

/*
//From https://stackoverflow.com/questions/785097/how-do-i-implement-a-b%C3%A9zier-curve-in-c/11435243#11435243
int getPt( int n1 , int n2 , float perc )
{
    int diff = n2 - n1;

    return n1 + ( diff * perc );
}

//Quadric bezier curve
for( float i = 0 ; i < 1 ; i += 0.01 )
{
    // The Green Line
    xa = getPt( x1 , x2 , i );
    ya = getPt( y1 , y2 , i );
    xb = getPt( x2 , x3 , i );
    yb = getPt( y2 , y3 , i );

    // The Black Dot
    x = getPt( xa , xb , i );
    y = getPt( ya , yb , i );

    drawPixel( x , y , COLOR_RED );
}

//Cubic bezier curve
for( float i = 0 ; i < 1 ; i += 0.01 )
{
    // The Green Lines
    xa = getPt( x1 , x2 , i );
    ya = getPt( y1 , y2 , i );
    xb = getPt( x2 , x3 , i );
    yb = getPt( y2 , y3 , i );
    xc = getPt( x3 , x4 , i );
    yc = getPt( y3 , y4 , i );

    // The Blue Line
    xm = getPt( xa , xb , i );
    ym = getPt( ya , yb , i );
    xn = getPt( xb , xc , i );
    yn = getPt( yb , yc , i );

    // The Black Dot
    x = getPt( xm , xn , i );
    y = getPt( ym , yn , i );

    drawPixel( x , y , COLOR_RED );
}
*/

Curve::Curve(my_point p0, my_point p1, my_point p2, my_point p3){
	this->p0 = p0;
	this->p1 = p1;
	this->p2 = p2;
	this->p3 = p3;
}

double Curve::get_frame_len(){
	double frame_len = distance(p0, p1) +
                       distance(p1, p2) +
					   distance(p2, p3);
    return frame_len;
}

bool Curve::validT(double t) {
	return 0<=t && t<=1;
}

vector<double> Curve::t_values_at_local_min_max(char dir) {
	double v0 = (dir == 'x') ? get_pN(0).x : get_pN(0).y;
	double v1 = (dir == 'x') ? get_pN(1).x : get_pN(1).y;
	double v2 = (dir == 'x') ? get_pN(2).x : get_pN(2).y;
	double v3 = (dir == 'x') ? get_pN(3).x : get_pN(3).y;
	double discrim = v1*v1 + v2*v2 + v0*(-v2+v3) - v1*(v2+v3);
	double top1 = v0 - 2.*v1 + v2;
	double bottom1 = v0 - 3.*v1 + 3.*v2 - v3;
	vector<double> roots;
	if(abs(bottom1) < 0.001) bottom1 = 0;
	if(bottom1 != 0) {
		double root1 = (top1 - sqrt(discrim))/bottom1;
		double root2 = (top1 + sqrt(discrim))/bottom1;
		if(abs(root1) < 0.001) root1 = 0;
		if(abs(root2) < 0.001) root2 = 0;
		if(abs(1-root1) < 0.001) root1 = 1;
		if(abs(1-root2) < 0.001) root2 = 1;
		// one real root
		if(discrim == 0) {
			if(validT(root1)) roots.push_back(root1);
		}
		// two real roots
		else if(discrim > 0) {
			if(validT(root1)) roots.push_back(root1);
			if(validT(root2)) roots.push_back(root2);
		}
	}
	else {
		double top2 = v0 - v1;
		double bottom2 = 2*(v0 - 2*v1 + v2);
		if(abs(bottom2) < 0.001) bottom2 = 0;
		if(bottom2 != 0) {
			double root = top2/bottom2;
			if(abs(root) < 0.001) root = 0;
			if(abs(1-root) < 0.001) root = 1;
			if(validT(root)) roots.push_back(root);
		}
		else {
			double top3 = 2.*v1 - v2;
			double bottom3 = 3.*(v1 - v2);
			if(abs(bottom3) < 0.001) bottom3 = 0;
			if(bottom3 != 0) {
				double root = top3/bottom3;
				if(abs(root) < 0.001) root = 0;
				if(abs(1-root) < 0.001) root = 1;
				if(validT(root)) roots.push_back(root);
			}
			else {
				// equation simplifies to v1, has no root
			}
		}
	}
	return roots;
}

// roots of ax^2 + bx + c = 0
vector<double> get_quadratic_roots(double a, double b, double c) {
	vector<double> roots;
	// if a==0, x = -c/b
	if(a == 0) {
		roots.push_back(-c/b);
	} else {
		// if a!=0
		double discrim = b*b-4*a*c;
		// one real root
		if(discrim == 0) {
			roots.push_back(-b/(2*a));
		} else if(discrim > 0) {
			// two real roots
			roots.push_back((-b-sqrt(discrim))/(2*a));
			roots.push_back((-b+sqrt(discrim))/(2*a));
		}
	}
	return roots;

}

//www.caffeineowl.com/graphics/2d/vectorial/cubic-inflexion.html
vector<double> Curve::t_values_at_inflection_points() {
	my_point P1 = get_pN(0), C1 = get_pN(1), C2 = get_pN(2), P2 = get_pN(3);
	my_point a = C1 - P1;
	my_point b = C2 - C1 - a;
	my_point c = P2 - C2 - a - b*2;
	// quadratic coefficients
	double coeff2 = b.x*c.y - b.y*c.x;
	double coeff1 = a.x*c.y - a.y*c.x;
	double coeff0 = a.x*b.y - a.y*b.x;
	// get roots
	vector<double> roots = get_quadratic_roots(coeff2, coeff1, coeff0);
	for(int i=roots.size()-1; i >= 0; i--) {
		double root = roots.at(i);
		if( !(root>0 && root<1) ){
			roots.erase(roots.begin()+i);
		}
	}
	return roots;
}

double rescale_to_interval(double val, double a, double b) {
	assert(a != b);
	return (val-a)/(b-a);
}

// extract the curve from parameter t1 to t2 (0 <= t1 < t2 <= 1)
Curve Curve::get_sub_curve(double t1, double t2) {
	//cout << "t1: " << t1 << " t2: " << t2 << endl;
	assert(validT(t1) && validT(t2) && t1<t2);
	// split at t1 first
	my_point P0 = get_pN(0), P1 = get_pN(1), P2 = get_pN(2), P3 = get_pN(3);
	my_point P01 = (P0*(1-t1)) + (P1*t1);
	my_point P12 = (P1*(1-t1)) + (P2*t1);
	my_point P23 = (P2*(1-t1)) + (P3*t1);
	my_point P012 = (P01*(1-t1)) + (P12*t1);
	my_point P123 = (P12*(1-t1)) + (P23*t1);
	my_point P0123 = (P012*(1-t1)) + (P123*t1);
	// two split curves
	//MyCurve part1 = new MyCurve(P0, P01, P012, P0123);
	Curve part2 = Curve{P0123, P123, P23, P3};

	// then split part2 at t2 (adjusted)
	double newT2 = rescale_to_interval(t2, t1, 1);
	my_point P0_ = part2.get_pN(0), P1_ = part2.get_pN(1), P2_ = part2.get_pN(2), P3_ = part2.get_pN(3);
	my_point P01_ = (P0_ * (1-newT2)) + (P1_ * newT2);
	my_point P12_ = (P1_ * (1-newT2)) + (P2_ * newT2);
	my_point P23_ = (P2_ * (1-newT2)) + (P3_ * newT2);
	my_point P012_ = (P01_ * (1-newT2)) + (P12_ * newT2);
	my_point P123_ = (P12_ * (1-newT2)) + (P23_ * newT2);
	my_point P0123_ = (P012_ * (1-newT2)) + (P123_ * newT2);
	// two split curves
	Curve part1_ = Curve{P0_, P01_, P012_, P0123_};
	//MyCurve part2_ = new MyCurve(P0123_, P123_, P23_, P3_);
	// return part1 of part2
	return part1_;
}

bool Curve::is_degenerate(){
	return get_frame_len() < 0.001;
}

bool Curve::is_straight(){
	double dist = length(p3 - p0);
    bool straight = abs(dist - get_frame_len()) < 0.01;
	return straight;
}

//TODO perhaps change all get_pN methods to just access the variable directly
bool Curve::is_min_at0(char dir) {
	double tol = 0.1;
	double v0 = (dir == 'x') ? get_pN(0).x : get_pN(0).y;
	double v1 = (dir == 'x') ? get_pN(1).x : get_pN(1).y;
	double v2 = (dir == 'x') ? get_pN(2).x : get_pN(2).y;
	double v3 = (dir == 'x') ? get_pN(3).x : get_pN(3).y;
	bool isMin = v0-v1 <= tol && v0-v2 <= tol & v0-v3 <= tol;
	return isMin;
}

bool Curve::is_min_at1(char dir) {
	double tol = 0.1;
	double v0 = (dir == 'x') ? get_pN(0).x : get_pN(0).y;
	double v1 = (dir == 'x') ? get_pN(1).x : get_pN(1).y;
	double v2 = (dir == 'x') ? get_pN(2).x : get_pN(2).y;
	double v3 = (dir == 'x') ? get_pN(3).x : get_pN(3).y;
	bool isMin = v3-v0 <= tol && v3-v1 <= tol & v3-v2 <= tol;
	return isMin;
}

bool Curve::is_max_at0(char dir) {
	double tol = 0.1;
	double v0 = (dir == 'x') ? get_pN(0).x : get_pN(0).y;
	double v1 = (dir == 'x') ? get_pN(1).x : get_pN(1).y;
	double v2 = (dir == 'x') ? get_pN(2).x : get_pN(2).y;
	double v3 = (dir == 'x') ? get_pN(3).x : get_pN(3).y;
	bool isMax = v0-v1 >= -tol && v0-v2 >= -tol & v0-v3 >= -tol;
	return isMax;
}

bool Curve::is_max_at1(char dir) {
	double tol = 0.1;
	double v0 = (dir == 'x') ? get_pN(0).x : get_pN(0).y;
	double v1 = (dir == 'x') ? get_pN(1).x : get_pN(1).y;
	double v2 = (dir == 'x') ? get_pN(2).x : get_pN(2).y;
	double v3 = (dir == 'x') ? get_pN(3).x : get_pN(3).y;
	bool isMax = v3-v0 >= -tol && v3-v1 >= -tol & v3-v2 >= -tol;
	return isMax;
}

my_point Curve::get_pN(int N){
	assert( N >= 0 && N < 4);
	switch (N) {
		case 0:
			return p0;
		case 1:
			return p1;
		case 2:
			return p2;
		default:
			return p3;
	}
}

my_point Curve::get_handle1(){
	return p1 - p0;
}

my_point Curve::get_handle2(){
	return p2 - p3;
}

my_point Curve::get_point(double t) {
	assert(validT(t));
	double r = 1-t;
	// B(t) = (1-t)^3*P0 + 3t(1-t)^2*P1 + 3(1-t)t^2*P2 + t^3*P3
	my_point A = p0 * (r*r*r);
	my_point B = p1 * (3*r*r*t);
	my_point C = p2 * (3*r*t*t);
	my_point D = p3 * (t*t*t);
	return A+B+C+D;
}

// output = {topleft, bottomright}
vector<my_point> Curve::get_bounding_box() {
	vector<double> txs = t_values_at_local_min_max('x');
	vector<double> tys = t_values_at_local_min_max('y');
	vector<double> ts;

	ts.insert(ts.end(), txs.begin(), txs.end());
	ts.insert(ts.end(), tys.begin(), tys.end());
	ts.push_back(1.0);
	double xmin = get_point(0).x, xmax = get_point(0).x,
		   ymin = get_point(0).y, ymax = get_point(0).y;

	for(size_t i=0; i < ts.size(); i++) {
		my_point p = get_point(ts.at(i));
		if(p.x < xmin){
			xmin = p.x;
		}
		if(p.x > xmax){
			xmax = p.x;
		}
		if(p.y < ymin){
			ymin = p.y;
		}
		if(p.y > ymax){
			ymax = p.y;
		}
	}
	vector<my_point> bb{my_point{xmin,ymin}, my_point{xmax, ymax}};
	return bb;
}

my_point Curve::get_tangent(double t) {
	assert(validT(t));
	// dx/dt = -3(1-t)^2*x0 + 3(1-t)(1-3t)*x1 + 3t(2-3t)*x2 + 3t^2*x3
	double A = -3*(1-t)*(1-t);
	double B = 3*(1-t)*(1-3*t);
	double C = 3*t*(2-3*t);
	double D = 3*t*t;
	double dxdt = A*p0.x + B*p1.x + C*p2.x + D*p3.x;
	double dydt = A*p0.y + B*p1.y + C*p2.y + D*p3.y;
	//sSystem.out.println(A + ", " + B + ", " + C + ", " + D + ", " + dxdt + ", " + dydt);
	return my_point{dxdt, dydt};
}
