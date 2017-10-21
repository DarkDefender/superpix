#include "segment.h"
#include <math.h>

Segment::Segment(){
	update_horz_and_vert();
}

Segment::Segment(my_point point){
	//Leave the handles as zero
	this->point = point;
	update_horz_and_vert();
}

Segment::Segment(my_point point, my_point handle_in, my_point handle_out){
	this->point = point;
	this->handle_in = handle_in;
	this->handle_out = handle_out;
	update_horz_and_vert();
}

void Segment::update_horz_and_vert(){
	this->horizontal = (abs(handle_in.y) < 0.001 && abs(handle_out.y) < 0.001);
	this->vertical = (abs(handle_in.x) < 0.001 && abs(handle_out.x) < 0.001);
}

my_point Segment::get_point(){
	return point;
}

my_point Segment::get_handle_in(){
	return handle_in;
}

my_point Segment::get_handle_out(){
	return handle_out;
}

void Segment::set_point(my_point point){
	this->point = point;
}

void Segment::set_handle_in(my_point handle_in){
	this->handle_in = handle_in;
}

void Segment::set_handle_out(my_point handle_out){
	this->handle_out = handle_out;
}

my_point transform_point(my_point p, Transform<double,2,Affine> at) {
	Vector2d v1(p.x, p.y);
	Vector2d v2 = at * v1;
	//at.transform(p, newPt);
	return my_point{v2[0], v2[1]};
}

my_point transform_vector(my_point p, Transform<double,2,Affine> at) {
	Vector2d v1(p.x, p.y);
	Vector2d v2 = at.linear() * v1;
	//at.deltaTransform(p, newPt);
	return my_point{v2[0], v2[1]};
}

void Segment::transform(Transform<double,2,Affine> at){
	point = transform_point(point, at);
	handle_in =	transform_vector(handle_in, at);
	handle_out = transform_vector(handle_out, at);
}

//Flag Seg

FlaggedSegment::FlaggedSegment() : Segment(){

}
FlaggedSegment::FlaggedSegment(my_point point, my_point handle_in, my_point handle_out) :
Segment(point, handle_in, handle_out)
{

}
void FlaggedSegment::set_flag(bool flag){
	this->flag = flag;
}

bool FlaggedSegment::get_flag(){
	return flag;
}
