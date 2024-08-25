#ifndef JOINT

#define JOINT

#include "pos.h"
#include "vector.h"
#include <iostream>
#include <string>
#include "matrix.h"
#include <vector>
#include <cmath>

#define PI 3.14159265

struct joint
{
	public:
		double angle;
		unsigned pos;
		Pos jpos;
		Vector axis;
		bool stat;
	
	public:
		joint() = default;
		joint(double d1, unsigned u1, Pos p1, Vector v1,bool b1) : angle(d1), pos(u1), jpos(p1), axis(v1) ,stat(b1) {};
		
	public:
		ostream& print(ostream& o1, joint& j1)
		{
			o1 << "angle: " << j1.angle << " pos: " << j1.pos << " jpos: ";
			j1.jpos.print(o1,j1.jpos);
			o1 << " axis: ";
			j1.axis.print(o1,j1.axis);
			
			return o1;
		}
		
	
	public:
		Vector unit_link_vector(Pos,Pos);
		Pos move_joint_pos(Pos,Vector,double);
		double xz_angle(Pos);
		Pos twod_to_3d_pos(double,Pos);
		Pos threed_to_2d_pos(double,Pos);
		Pos vector_to_pos(Vector);
		Vector pos_to_vector(Pos);

};



Vector joint::unit_link_vector(Pos p1, Pos p2)
{
	Vector v = p1.vect_sub(p1,p2);
	v = v.normal(v);
	return v;
}

Pos joint::move_joint_pos(Pos p1,Vector v1,double length)
{
	Vector vs = v1.scale(v1,length);
	Pos jp = p1.pos_vect_add(p1,vs);
	return jp;
}

double joint::xz_angle(Pos target)
{
	double r = (target.z/target.x);
	double angle = atan(r);
	
	if(target.x > 0)
	{
		if(angle >= 0)
		{
			angle = (angle*180/PI);
		}
		
		else
		{
			angle = (angle*180/PI)+360;
		}
	}
	
	else
	{
		angle = (angle*180/PI)+180;
	}
	
	return angle;
}

Pos joint::twod_to_3d_pos(double angle,Pos p)
{
	Vector v(0,1,0);
	matrix m;
	m = m.rotation_matrix(-angle,v);
	
	Vector vp = joint::pos_to_vector(p);
	Vector twod_vp = m.vector_matrix_product(m,vp);
	Pos twod_p = joint::vector_to_pos(twod_vp);
	
	return twod_p;
}


Pos joint::threed_to_2d_pos(double angle,Pos p)
{
	Vector v(0,1,0);
	matrix m;
	m = m.rotation_matrix(angle,v);
	
	Vector vp = joint::pos_to_vector(p);
	Vector threed_vp = m.vector_matrix_product(m,vp);
	Pos threed_p = joint::vector_to_pos(threed_vp);
	
	return threed_p;
}

Pos joint::vector_to_pos(Vector v1)
{
	Pos p;
	p.x = v1.x;
	p.y = v1.y;
	p.z = v1.z;
	
	return p;
}

Vector joint::pos_to_vector(Pos p1)
{
	Vector v;
	v.x = p1.x;
	v.y = p1.y;
	v.z = p1.z;
	
	return v;
}



#endif
