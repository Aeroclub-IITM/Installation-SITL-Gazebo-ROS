#!/usr/bin/env python
# This code is to make drone move around in a circle.
# Headers
import rospy
import math
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped

current_pos = PoseStamped()

# Just showing how you can the values by subscribing 
def current_pos_callback(position):

    global current_pos
    current_pos = position

#Starting a node
def circle():
     rospy.init_node('make_a_circle', anonymous=True)
     publish_velocity=rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped,queue_size=20)
     vel=TwistStamped()

#Getting input
     radius=float(input("Enter radius of the circle    "))
     speed=float(input("Enter the speed of the robot    "))
     r=abs(radius)
     omega=speed/r
     
     while(current_pos.pose.position.x<radius):
        rospy.Subscriber('mavros/local_position/pose',PoseStamped,current_pos_callback)
        vel.twist.linear.x=speed
        vel.twist.linear.y=0
        vel.twist.linear.z=0
        publish_velocity.publish(vel)
        
     rate = rospy.Rate(10)
     while not rospy.is_shutdown():
         t0=rospy.Time.now().to_sec()
         ang=0
         
#Loop to move in complete circle
         while(ang<math.pi*2):
             rate.sleep()
             rospy.Subscriber('mavros/local_position/pose',PoseStamped,current_pos_callback)
             print(ang)
             vel.twist.linear.x= -math.sin(ang)*speed
             vel.twist.linear.y= math.cos(ang)*speed
             vel.twist.linear.z=0
             publish_velocity.publish(vel)
             t1=rospy.Time.now().to_sec() #current time
             ang=omega*(t1-t0)
             
         vel.twist.angular.x=0
         vel.twist.angular.y=0
         vel.twist.angular.z=0
         publish_velocity.publish(vel)

if __name__ == '__main__':
     try:
          circle()
     except rospy.ROSInterruptException:
          pass

