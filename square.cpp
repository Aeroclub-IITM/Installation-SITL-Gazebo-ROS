// Program to execute a path of square

// Header files

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>

#include <geometry_msgs/Twist.h>

#include <math.h>

// Variable Defenitions

geometry_msgs::PoseStamped setpoint;

geometry_msgs::PoseStamped current_position;

geometry_msgs::PoseStamped pos_pub;

float xi, yi, zi;

float x[]={5.0,-5.0,-5.0,5.0};

float y[]={5.0,5.0,-5.0,-5.0};

float z[]={7.0,7.0,7.0,7.0};

int n,i=0;

//Function Prototypes

void setpoint_pub(float p,float q ,float r);

int wp_change(float p,float q,float r);

//Publisher Client

ros::Publisher setpoint_client;


//Subscriber Client

ros::Subscriber current_pos_client;
ros::Subscriber reached_setpos_client;


//Call_Back function to get the current position values.

void current_pos_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)

{

  current_position=*msg;

  ROS_INFO("Flag 3");

}


// Main function

int main(int argc, char **argv)

{
	// Initializing the node

    ros::init(argc, argv, "wp_node");

    ros::NodeHandle nh;

    //Publisher Clients

    setpoint_client = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",20);


	//Subscriber Client

	current_pos_client = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1000, current_pos_callback);


	ROS_INFO("Flag 1 \n");

    ros::Rate rate(20.0);

    setpoint_pub(x[i],y[i],z[i]);

	while(ros::ok())

	{	// Just after executing the node may not start publishing values , resulting in omitting first waypoint. In order to avoid that we are publishing many times. 
		while(i==0)
		{
			setpoint_pub(x[i],y[i],z[i]);

		}			

		std::cout<<"setpoint given";

		xi=current_position.pose.position.x;

		yi=current_position.pose.position.y;

		zi=current_position.pose.position.z;
		
		// For trajectory planning

		wp_change(xi,yi,zi);

		ROS_INFO("Flag 2");

		ros::spinOnce();

		rate.sleep();

	}

    return(0);

} 


//Function Defenitions 

int wp_change(float p,float q,float r)

{

	float dist;

	dist=((x[i]-p)*(x[i]-p)+(y[i]-q)*(y[i]-q));

	std::cout<<"WAYPOINTS ..\n x is "<< x[i] <<"\n y is "<< y[i]<<" \n z is "<<z[i]<<"\n";

	std::cout<<"Current_position ..\n x is "<< p <<"\n y is "<< q<<" \n z is "<<r<<"\n";

	std::cout<<"Error is "<<dist<<"\n";        

	// Defining an eror circle of 1 units.

	if(dist<1.0)

	{

		i++;

		std::cout<<"\n\n\n\n\n\n\n waypoint change happened, new i is"<<i<<"\n\n";



		if(i>3)

			i=0;
		
		
	setpoint_pub(x[i],y[i],z[i]);
	}


        std::cout<<x[i];
	return(0);





}

void setpoint_pub(float p, float q, float r)

{
	pos_pub.pose.position.x = p;

	pos_pub.pose.position.y = q;

	pos_pub.pose.position.z = r;

	pos_pub.pose.orientation.x = 0.0;

	pos_pub.pose.orientation.y = 0.0;

	pos_pub.pose.orientation.z = 0.0;

	pos_pub.pose.orientation.w = 1.0;

	// Publishing points.
	
    setpoint_client.publish(pos_pub);
	
}




