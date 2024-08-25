#ifndef VECTOR

#define VECTOR

#include <iostream>
#include <string>
#include <cmath>
#include "pos.h"

using std::cout;
using std::cin;
using std::endl;
using std::ostream;
using std::istream;


struct Vector
{
	public:
		double x;
		double y;
		double z;
		
	public:
		Vector(double d1 = 0,double d2 = 0, double d3 = 0) : x(d1), y(d2), z(d3) {}
		
	public:
		ostream& print(ostream& o1, Vector& v1)
		{
			o1 << "x: " << v1.x << " y: " << v1.y << " z: " << v1.z;
			return o1;
		}
		
	
	public:
		double magnitude(Vector);
		Vector normal(Vector);
		Vector scale(Vector, double);
		Vector cross_prod(Vector,Vector);
		Vector round_vect(Vector);
};


double Vector::magnitude(Vector v1)
{
	double m = (v1.x*v1.x + v1.y*v1.y + v1.z*v1.z);
	return sqrt(m);
}



Vector Vector::normal(Vector v1)
{
	double m = v1.magnitude(v1);
	Vector v;
	v.x = (v1.x/m);
	v.y = (v1.y/m);
	v.z = (v1.z/m);
	
	return v;
}

Vector Vector::scale(Vector v1, double s)
{
	Vector v;
	v.x = (v1.x*s);
	v.y = (v1.y*s);
	v.z = (v1.z*s);
	
	return v;
}

Vector Vector::cross_prod(Vector v1, Vector v2)
{
	Vector v;
	
	v.x = (v1.y*v2.z - v2.y*v1.z);
	v.y = -(v1.x*v2.z - v2.x*v1.z);
	v.z = (v1.x*v2.y - v2.x*v1.y);	
	return v;
}

Vector Vector::round_vect(Vector v1)
{
	Vector v;
	v.x = round(v1.x*1000)/1000;
	v.y = round(v1.y*1000)/1000;
	v.z = round(v1.z*1000)/1000;
	
	return v;
}






#endif
