#ifndef MATRIX

#define MATRIX
#define PI 3.14159265

#include <vector>
#include <iostream>
#include <string>
#include "vector.h"
#include <cmath>

using std::cout;
using std::cin;
using std::endl;
using std::ostream;
using std::istream;


struct matrix
{
	
	public:
		double m[3][3];
	
	
	public:
		matrix(double r1[3], double r2[3], double r3[3])
		{
			for(int i = 0; i != 3; i++)
			{
				m[0][i] = r1[i];
			}
			
			for(int i = 0; i != 3; i++)
			{
				m[1][i] = r2[i];
			}
			
			for( int i =0; i != 3; i++)
			{
				m[2][i] = r3[i];
			}
		}
		
		matrix() = default;
	
	public:
		ostream& print(ostream& o1, matrix& m1)
		{
			for(int j = 0; j !=3; j++)
			{
				for(int i=0; i != 3; i++)
				{
					o1 << m1.m[j][i] << " ";
				}
				
				o1 << endl;
			}
			
			return o1;	
		}
		
	
	public:
		Vector vector_matrix_product(matrix,Vector);
		matrix rotation_matrix(double,Vector);
	
};


Vector matrix::vector_matrix_product(matrix m1, Vector v1)
{
	Vector v;
	
	v.x = v1.x*m1.m[0][0] + v1.y*m1.m[0][1] + v1.z*m1.m[0][2];
	v.y = v1.x*m1.m[1][0] + v1.y*m1.m[1][1] + v1.z*m1.m[1][2];
	v.z = v1.x*m1.m[2][0] + v1.y*m1.m[2][1] + v1.z*m1.m[2][2];	
	
	return v;
}


matrix matrix::rotation_matrix(double angle, Vector v1)
{
	matrix m1;
	
	m1.m[0][0] = cos(angle*PI/180) + v1.x*v1.x * (1 - cos(angle*PI/180));
	m1.m[1][0] = v1.y*v1.x*(1 - cos(angle*PI/180)) + v1.z*sin(angle*PI/180);
	m1.m[2][0] = v1.z*v1.x*(1 - cos(angle*PI/180)) - v1.y*sin(angle*PI/180);
	
	m1.m[0][1] = v1.x*v1.y*(1 - cos(angle*PI/180)) - v1.z*sin(angle*PI/180);
	m1.m[1][1] = cos(angle*PI/180) + v1.y*v1.y*(1 - cos(angle*PI/180));
	m1.m[2][1] = v1.z*v1.y*(1 - cos(angle*PI/180)) + v1.x * sin(angle*PI/180);
	
	
	m1.m[0][2] = v1.z*v1.x*(1 - cos(angle*PI/180)) + v1.y*sin(angle*PI/180); 
	m1.m[1][2] = v1.y*v1.z*( 1 - cos(angle*PI/180) ) - v1.x*sin(angle*PI/180);
	m1.m[2][2] = cos(angle*PI/180) + v1.z*v1.z* (1 - cos(angle*PI/180));
	
	return m1;	
}









#endif
