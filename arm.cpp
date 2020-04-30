#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>

#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <math.h>
#include <std_msgs/Int8.h>

// Service Cients
ros::ServiceClient mode_change_client;
ros::ServiceClient takeoff_client;
ros::ServiceClient arming_client;


//Function Prototypes
void mode_change(int mode);
void takeoff(mavros_msgs::CommandTOL takeoff_points);
void arm(bool state);



//variable defenitions
mavros_msgs::CommandTOL points;
mavros_msgs::SetMode mod;
char yesno;
bool success;



int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    
    // Service Clients
    
    mode_change_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
	
	
	mode_change(4);
	arm(true);
    std::cout<<"/n"<<" DO YOU WANNA TAKEOFF y/n ... \n";
    std::cin>>yesno;
    if(yesno=='y')
    {
		std::cout<<"ALTITUDE - 10 \n";
		points.request.altitude=10.0;
		// std::cin>>points.altitude;
		takeoff(points);	   
    }
    else
    	std::cout<<"ok bye";
    std::cout<<"do ";
    return(0);
    
 
    
}

//functions


void mode_change(int mode)
{
	mod.request.custom_mode=(char)(48+mode);
	mod.request.base_mode=0;
	mode_change_client.call(mod);
	std::cout<<mod.response.mode_sent<<" response ";

}
void takeoff(mavros_msgs::CommandTOL takeoff_points)
{
	takeoff_client.call(takeoff_points);
}
void arm(bool state)
{
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = state;
	arming_client.call(arm_cmd);
	std::cout<<" \n ARMED \n";
}
  
 
