#ifndef ARM
#define ARM

#define PI 3.14159265

#include "pos.h"
#include "vector.h"
#include "matrix.h"
#include "Joint.h"
#include "Link.h"

#include <string>
#include <vector>
#include <iostream>

using std::string;
using std::cout;
using std::cin;
using std::endl;
using std::ostream;
using std::istream;
using std::vector;


struct arm
{
	public:
		vector<joint> jlist;
		vector<link> llist;
		double threshold;
		Pos target;
	
	public:
		arm() = default;
		arm(vector<joint> v1, vector<link> v2, double d1,Pos p1) : jlist(v1),llist(v2),threshold(d1),target(p1) {}
		
	public:
		ostream& print(ostream& o1, arm& a1)
		{
			o1 << "Joint list: " << endl;
			for(auto it = a1.jlist.begin(); it != a1.jlist.end(); ++it)
			{
				it->print(o1,*it);
				o1 << endl;
			}
			
			o1 << "Link list: " << endl;
			for(auto it = a1.llist.begin(); it != a1.llist.end(); ++it)
			{
				it->print(o1,*it);
				o1 << endl;
			}
			
			o1 << "Threshold: " << endl;
			o1 << a1.threshold << endl;
			
			o1 << "Target: " << endl;
			a1.target.print(o1,a1.target);
			o1 << endl;
			
			return o1;
		}
		
	public:
		bool error_check(Pos,Pos,double);
		Pos fetch_target(istream&);
		
		vector<Pos> forward_backward_pass(vector<joint>, vector<link>, Pos);
		vector<Pos> threed_to_2d_translate(vector<Pos>, Pos);
		vector<Pos> twod_to_3d_translate(vector<Pos>,Pos);
		vector<joint> root_arm(vector<joint>,vector<link>);
		vector<Pos> move_end_effector(vector<joint>,vector<link>,Pos,double);


};

	
	bool arm::error_check(Pos p1, Pos p2, double thresh)
	{
		Vector vdiff = p1.vect_sub(p1,p2);
		double error = vdiff.magnitude(vdiff);
		
		return (error < thresh);
	}
	
	
	Pos arm::fetch_target(istream& in)
	{
		Pos p1;
		in >> p1.x >> p1.y >> p1.z;
		return p1;
	}
	
	vector<Pos> arm::forward_backward_pass(vector<joint> jl, vector<link> ll, Pos target)
	{
		jl[jl.size()].jpos = target;
		vector<Pos> plist;
		
		
		for(int i =jl.size(), j = ll.size(); i != 1; i--,j--)
		{
			if(jl[i].stat == true)
			{
				break;
			}
			
			Vector uv = jl[i].unit_link_vector(jl[i].jpos,jl[i-1].jpos);
			Pos jp = jl[i].move_joint_pos(jl[i].jpos,uv,ll[j].length);
			jl[i-1].jpos = jp;
			this->jlist[i-1].jpos = jp;
		}
		
		
		int count = -1;
		for(int i = 0; i < jl.size(); i++)
		{
			if(jl[i].stat == true)
			{
				jl[i].jpos = Pos(0,0,0);
				this->jlist[i].jpos = Pos(0,0,0);
				count++;
			}
			
			else
			{
				i = jl.size();
			}
		}
		
		
		
		
		for(int i = count, j = 0; i != jl.size()-1; i++,j++)
		{
			Vector uv = jl[i].unit_link_vector(jl[i].jpos, jl[i+1].jpos);
			Pos jp = jl[i].move_joint_pos(jl[i].jpos,uv,ll[j].length);
			jl[i+1].jpos = jp;
			this->jlist[i+1].jpos = jp;
		}
		
		
		
		for(auto it = jl.begin(); it != jl.end(); it++)
		{
			plist.push_back(it->jpos);
		}
		
		
		return plist;
	}
	
	
	vector<Pos> arm::threed_to_2d_translate(vector<Pos> pl, Pos target)
	{
		double angle = this->jlist[0].xz_angle(target);
		
		for(int i = 0; i != pl.size(); i++)
		{
			pl[i] = this->jlist[0].threed_to_2d_pos(angle,pl[i]);
			pl[i] = pl[i].round_Pos(pl[i]);
			this->jlist[i].jpos = pl[i];
		}
		
		return pl;
	}
	
	vector<Pos> arm::twod_to_3d_translate(vector<Pos> pl, Pos target)
	{
		double angle = this->jlist[0].xz_angle(target);
		
		for(int i = 0; i !=pl.size(); i++)
		{
			pl[i] = this->jlist[0].twod_to_3d_pos(angle,pl[i]);
			pl[i] = pl[i].round_Pos(pl[i]);
			this->jlist[i].jpos = pl[i];
		}
		return pl;
	}
	
	vector<joint> arm::root_arm(vector<joint> jl, vector<link> ll)
	{
		int count = 0;
		for(int i=0; i != jl.size(); i++)
		{
			if(jl[i].stat)
			{
				jl[i].jpos = Pos(0,0,0);
				this->jlist[i].jpos = Pos(0,0,0);
				count ++;
			}
			
			else
			{
				break;
			}
		}
		
		
		
		for(int j = count, k = 0; j != jl.size(); j++,k++)
		{
			double yp = jl[j-1].jpos.y + ll[k].length;
			jl[j].jpos = Pos(0,yp,0);
			this->jlist[j].jpos = Pos(0,yp,0);
		}
		
		return jl;
	}
	
	
	vector<Pos> arm::move_end_effector(vector<joint> jl, vector<link> ll, Pos target, double th)
	{
		jl = this->root_arm(jl,ll);
		
		vector<Pos> pl = {target};
		Pos target_2d =  this->threed_to_2d_translate(pl,target)[0];
		vector<joint> jl_ = jl;
		vector<link> ll_ = ll;
		
		
		do
		{
			this->forward_backward_pass(jl_, ll_, target_2d);
		
		} while (this->error_check(target_2d, this->jlist[ this->jlist.size()].jpos, th));
		
		
		vector<Pos> pl_;
		for(auto iter = this->jlist.begin(); iter != this->jlist.end(); iter++)
		{
			pl_.push_back(iter->jpos);
			
		}
		
		vector<Pos> pl__ = this->twod_to_3d_translate(pl_,target);
		return pl__;	
	}
	
	//vector<Pos> pl = {target};
	//	Pos target_2d =  this->threed_to_2d_translate(pl,target)[0];
		
		//do
		//{
		//	this->forward_backward_pass(this->jlist, this->llist, target_2d);
		
		//} while (this->error_check(target_2d, this->jlist[ this->jlist.size()].jpos, threshold));
		
		
		//vector<Pos> pl_;
		//for(auto iter = this->jlist.begin(); iter != this->jlist.end(); iter++)
		//{
			//iter->jpos.print(r,iter->jpos);
			//r << endl;
		//	pl_.push_back(iter->jpos);
		//}
		
		
		//vector<Pos> jl_ = this->twod_to_3d_translate(pl_,target);
		//return jl_;


#endif
