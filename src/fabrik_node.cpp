#include "Arm.h"
#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "control_msgs/msg/multi_dof_command.hpp"
#include "geometry_msgs/msg/vector3.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;
using std::string;
using std::cout;
using std::endl;
using std::ostream;

class ControllerPublisher : public rclcpp::Node
{
  public:
    ControllerPublisher()
    : Node("controller_publisher"), count_(0)
    {
    
      //4 publishers for the 4 pid controllers
      publisher_1 = this->create_publisher<control_msgs::msg::MultiDOFCommand>("/controller1/reference", 10);
      publisher_2 = this->create_publisher<control_msgs::msg::MultiDOFCommand>("/controller2/reference", 10);
      publisher_3 = this->create_publisher<control_msgs::msg::MultiDOFCommand>("/controller3/reference", 10);
      publisher_4 = this->create_publisher<control_msgs::msg::MultiDOFCommand>("/controller4/reference", 10);

      subscription_1 = this->create_subscription<geometry_msgs::msg::Vector3>(
      "/target_pos", 10, std::bind(&ControllerPublisher::topic_callback, this, _1));
      
      //axes of joints
      Vector ax1(0,0,1);
      Vector ax2(cos(0*PI/180),sin(0*PI/180),0);
      Vector ax3 = ax2.cross_prod(ax2,ax1);
	
      //joints
      joint j1(0,1,Pos(0,0,0),ax1,true); 
      joint j2(0,2,Pos(1,0,0),ax3,true);
      joint j3(0,3,Pos(0,0,1.0),ax3,false);
      joint j4(0,4,Pos(0,0.0,1.5),ax3,false);
      joint j5(0,5,Pos(0.0,0.0,1.75),ax3,false);
      vector<joint> jl = {j1,j2,j3,j4,j5};
      
      //links
      fabrik::link l1(1,1,j3,j2);
      fabrik::link l2(0.5,2,j4,j3);
      fabrik::link l3(0.25,3,j5,j4);
      vector<fabrik::link> ll = {l1,l2,l3};
	
      
      //initializing threshold
      thresh = 0.01;
      
      //initializing target pos
      target = Pos(0.5,0.8,0.5);
      
      //initializing the arm object
      a1 = arm(jl,ll,thresh,target);
      
      //vector<double> ang = {0.0,0.0,0.0,0.0};


    }

  public:

    void topic_callback(const geometry_msgs::msg::Vector3 & msg)
    {
     //calling move_end_effector too apply fabrik algorithm, storing positions computed
     vector<Pos> pl;
     this->target = Pos(msg.x,msg.y,msg.z);
     cout << "target: " << this->target.x << " " << this->target.y << " " << target.z <<endl;
     
     //axis before debug
     cout << "axis before: " << a1.jlist[2].axis.x << " " << a1.jlist[2].axis.y << " " << a1.jlist[2].axis.z << endl;
     pl = this->a1.move_end_effector(this->a1.jlist,this->a1.llist,this->target,this->a1.threshold);
     
     ostream &r = cout;
     r << "Joint positions p1,p2,p3,p4,p5 after the move_end_effector() function are: " << endl;
     for(auto it = a1.jlist.begin(); it != a1.jlist.end(); it++)
     {
     	it->print(r,*it);
	r << endl;
     }
     
     //calling the move_end_effector angles function too get angle of joints
     vector<double> angles;
     angles = this->a1.move_end_effector_angles(this->a1.jlist);
     
     //axis after debug
     cout << "axis after: " << a1.jlist[2].axis.x << " " << a1.jlist[2].axis.y << " " << a1.jlist[2].axis.z << endl;
     
     auto mdof_1 = control_msgs::msg::MultiDOFCommand();
     mdof_1.dof_names = {"l2_l3"};
     mdof_1.values = {angles[3]};
     mdof_1.values_dot = {0.0};
     
     auto mdof_2 = control_msgs::msg::MultiDOFCommand();
     mdof_2.dof_names = {"l1_l2"};
     mdof_2.values = {angles[2]};
     mdof_2.values_dot = {0.0};
     
     auto mdof_3 = control_msgs::msg::MultiDOFCommand();
     mdof_3.dof_names = {"base_l1"};
     mdof_3.values = {-angles[1]};
     mdof_3.values_dot ={0.0};
     
     auto mdof_4 = control_msgs::msg::MultiDOFCommand();
     mdof_4.dof_names = {"world_base"};
     mdof_4.values = {angles[0]};
     mdof_4.values_dot = {0.0};
     
     publisher_1->publish(mdof_1);
     publisher_2->publish(mdof_2);
     publisher_3->publish(mdof_3);
     publisher_4->publish(mdof_4);
    }
    
    //adding the publishers and subscribers
    rclcpp::Publisher<control_msgs::msg::MultiDOFCommand>::SharedPtr publisher_1;
    rclcpp::Publisher<control_msgs::msg::MultiDOFCommand>::SharedPtr publisher_2;
    rclcpp::Publisher<control_msgs::msg::MultiDOFCommand>::SharedPtr publisher_3;
    rclcpp::Publisher<control_msgs::msg::MultiDOFCommand>::SharedPtr publisher_4;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscription_1;
    size_t count_;
    
    //adding the links, joints, axes vectors,threshhold, target, and arm object too Class members
    Vector ax1;
    Vector ax2;
    Vector ax3;
    
    joint j1; 
    joint j2;
    joint j3;
    joint j4;
    joint j5;
    vector<joint> jl;
    
    fabrik::link l1;
    fabrik::link l2;
    fabrik::link l3;
    vector<fabrik::link> ll;
    
    double thresh;
    
    Pos target;
    
    arm a1;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  //create a publisher ControllerPublisher object
  rclcpp::spin(std::make_shared<ControllerPublisher>());
  rclcpp::shutdown();
  return 0;
}

