#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#define BURGER_MAX_ANG_VEL 2.84

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rotate_robot", ros::init_options::AnonymousName);
    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    geometry_msgs::Twist twist;
    while (ros::ok())
    {
        twist.angular.z = BURGER_MAX_ANG_VEL;
		pub.publish(twist);
    }
}