#ifndef LINK
#define LINK

#include <iostream>
#include <string>
#include "pos.h"
#include "vector.h"
#include "matrix.h"
#include "Joint.h"
#include <vector>
#include <cmath>
#define PI 3.14159265


struct link
{

	public:
		double length;
		unsigned pos;
		joint child;
		joint parent;
	
	
	public:
		link() = default;
		link(double d1, unsigned u1, joint j1, joint j2) : length(d1),pos(u1),child(j1),parent(j2) {}
	
		
	public: 
		ostream& print(ostream& o1, link& l1)
		{
			o1 << "length: " << l1.length << " pos: " << l1.pos << " child: ";
			l1.child.print(o1,l1.child);
			o1 << " parent: ";
			l1.parent.print(o1,l1.parent);
			
			return o1;
		}

};


#endif
