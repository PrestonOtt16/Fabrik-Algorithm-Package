#ifndef POS

#define POS

#include <iostream>
#include <string>
#include "vector.h"

using std::cout;
using std::cin;
using std::endl;
using std::ostream;
using std::istream;

struct Pos
{

	public:
		double x;
		double y;
		double z;
	
	public:
		Pos(double d1 = 0, double d2 = 0, double d3 = 0) : x(d1), y(d2), z(d3) {}
		
	public:
		ostream& print(ostream& o1,Pos& p1)
		{
			o1 << "x: " << p1.x << " y: " << p1.y << " z: " << p1.z;
			return o1;
		}
		
	public:
		Vector vect_sub(Pos,Pos);
		Pos pos_vect_add(Pos,Vector);
		Pos round_Pos(Pos);

};


Vector Pos::vect_sub(Pos p1,Pos p2)
{
	Vector v;
	v.x = (p2.x - p1.x);
	v.y = (p2.y - p1.y);
	v.z = (p2.z - p1.z);
	
	return v;
}

Pos Pos::pos_vect_add(Pos p1, Vector v1)
{
	Pos p;
	
	p.x = (p1.x+v1.x);
	p.y = (p1.y+v1.y);
	p.z = (p1.z+v1.z);
	
	return p;
}

Pos Pos::round_Pos(Pos p1)
{
	Pos p;
	p.x = round(p1.x*1000)/1000;
	p.y = round(p1.y*1000)/1000;
	p.z = round(p1.z*1000)/1000;
	
	return p;
}


#endif
