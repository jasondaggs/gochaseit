#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"


class DriveBot
{

private:
	ros::NodeHandle nodeHandle;	
	ros::Publisher motorCommandPublisher; 
	ros::ServiceServer service;

public:

DriveBot()
{
	motorCommandPublisher = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

	service = nodeHandle.advertiseService("/ball_chaser/command_robot", &DriveBot::handle_command_robot_request,this);
                ROS_INFO("Dirve Bot:  Driving...");

	ROS_INFO("Drive Bot Initialized");
}

bool handle_command_robot_request(ball_chaser::DriveToTarget::Request &request, ball_chaser::DriveToTarget::Response &response)
{
	// Build the motor command from the request.
        geometry_msgs::Twist motorCommand;
        motorCommand.linear.x = request.linear_x;
        motorCommand.angular.z = request.angular_z;

	// Publish the motor command.
        motorCommandPublisher.publish(motorCommand);

	// Log what was published.
	ROS_INFO("DriveToTargetRequest received - linear_x:%1.2f, angular.z:%1.2f", (float)request.linear_x, (float)request.angular_z);
	
	response.msg_feedback = "Twist messasge set - linear_x: " + std::to_string(request.linear_x) + " , angular_z: " + std::to_string(request.angular_z);

        ROS_INFO_STREAM(response.msg_feedback);
	
	return true;
}
};

int main(int argc, char** argv)
{
	// Initialize a ROS node
	ros::init(argc, argv, "drive_bot");

	// Initialize and run the code for the service.
	DriveBot driveBot;
	ROS_INFO("Ready to service motor command");

	//Run main event loop.
	ros::spin();

	return 0;
}

