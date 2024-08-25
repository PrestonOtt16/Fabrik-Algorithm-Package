#include "Arm.h"

using std::string;
using std::cout;
using std::endl;
using std::ostream;


int main()
{
	Vector ax1(0,1,0);
	Vector ax2(cos(45*PI/180),0,sin(45*PI/180));
	Vector ax3 = ax2.cross_prod(ax2,ax1);
	
	joint j1(45,1,Pos(0,0,0),ax1,true); 
	joint j2(45,2,Pos(0,0,0),ax3,true);
	joint j3(-30,3,Pos(0.5,0.7071,0.5),ax3,false);
	joint j4(90,4,Pos(0.8415,0.8365,0.8415),ax3,false);
	joint j5(0,5,Pos(0.795,1.078,0.795),ax3,false);
	vector<joint> jl = {j1,j2,j3,j4,j5};
	
	link l1(1,1,j3,j2);
	link l2(0.5,2,j4,j3);
	link l3(0.25,3,j5,j4);
	vector<link> ll = {l1,l2,l3};
	
	double thresh = 0.1;
	Pos target(0.5,0.8,0.5);
	
	arm a1(jl,ll,thresh,target);
	vector<Pos> pl;
	pl = a1.move_end_effector(jl,ll,target,thresh);
	
	ostream &r = cout;
	r << "Joint positions p1,p2,p3,p4,p5 after the move_end_effector() function are: " << endl;
	for(auto it = pl.begin(); it != pl.end(); it++)
	{
		it->print(r,*it);
		r << endl;
	} 

}
