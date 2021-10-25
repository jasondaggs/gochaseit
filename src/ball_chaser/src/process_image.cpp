
#include <cstdlib>

#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

class ProcessImage
{

private:

ros::ServiceClient serviceClient;
ros::Subscriber sub1;
ros::NodeHandle n;

int speed = 2;

public:

ProcessImage()
{
    // Define a client service capable of requesting services from command_robot
    serviceClient = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    sub1 = n.subscribe("/camera/rgb/image_raw", 10, &ProcessImage::process_image_callback,this);
    ROS_INFO("Process image: ready for images");
}

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    ball_chaser::DriveToTarget driveToTarget;

    driveToTarget.request.linear_x = lin_x;
    driveToTarget.request.angular_z = ang_z;

    ROS_INFO("Calling DriveToTarget");
    serviceClient.call(driveToTarget);
   	 
} 

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    int white_pixel = 255;

    int min_x = img.step + 1;
    int max_x = -1; 
    int min_y = img.height + 1;
    int max_y = -1;

     for ( int y = 0; y < img.height; y++)
     {
	for ( int x = 0; x < img.step; x++)
        {
		int offset = ((y * img.step) + x);
		int pixel1 = img.data[offset];
		//int pixel2 = img.data[offset + ++x];
		//int pixel3 = img.data[offset + ++x];

		//if ( pixel1 == white_pixel && pixel2 == white_pixel && pixel3 == white_pixel)
		if ( pixel1 == white_pixel)
		{

		    if ( x < min_x)
			min_x = x;

		    if ( x > max_x)
			max_x = x;

		    if ( y < min_y)
			min_y = y;
			
		    if ( y > max_y)
			max_y = y;

		}	
	  }
     }

    int angular_z = calculate_angular_z(min_x,max_x,min_y,max_y,img);// * 5;
    int linear_x = calculate_linear_x(min_x,max_x,min_y,max_y,img);// * 1;

    drive_robot(linear_x,angular_z);
}

int calculate_linear_x(int min_x, int max_x, int min_y,int max_y,const sensor_msgs::Image img)
{
    // Find the width an height of the object.
    int object_width = std::abs(max_x/3 - min_x/3); //Divide by 3 to translate from step size.
    int object_height = std::abs(max_y - min_y);

    // Find the size of the image and object.
    float object_size = object_width * object_height;
    float image_size = img.width * img.height;


    //  Do we know if th ball exists.
    bool ball_exists = is_ball_visible(min_x,max_x,min_y,max_y,img);

	if (ball_exists)
	{
    		ROS_INFO("Ball Found");
	}

    
    // The size ratio tells is in relative terms how close the ball is to the camera.
    float size_ratio = object_size / image_size;

	std::cout << "size_ratio:"  << size_ratio << std::endl;


    if ( !ball_exists)
        return 0;

    if ( size_ratio < 0.07 )
        return 1;

    if ( size_ratio > 0.07 && size_ratio < 0.08)
	return 0;

    if (size_ratio > 0.09)
	return -1;

    return 0;
}

int calculate_angular_z(int min_x, int max_x, int min_y,int max_y,const sensor_msgs::Image img)
{
    //Find the ball width and height.
    int object_width = std::abs(max_x/3 - min_x/3);
    int object_height = std::abs(max_y - min_y);

	// Find the center of the ball.
    int center_x = min_x + object_width / 2;
    int center_y = min_y + object_height / 2;

    int angular_z = speed;
    bool ball_exists = is_ball_visible(min_x,max_x,min_y,max_y,img);

	// The ball is in the left side of the screen so rotate left.
    if (ball_exists && center_x < img.step/3)
        angular_z = speed; 

	// the ball is in the center of the screen so don't rotate at all.
    if (ball_exists &&  center_x > img.step/3 && center_x < img.step - img.step/3  )
        angular_z = 0;
	
	//  The ball is in the right side of the creen so rotate right.    
    if (ball_exists && center_x > img.step - img.step/3)
        angular_z = -speed; 
    
    return angular_z;
}

bool is_ball_visible(int min_x, int max_x, int min_y,int max_y,const sensor_msgs::Image img)
{

    int object_width = std::abs(max_x - min_x);
    int object_height = std::abs(max_y - min_y);

//	std::cout << min_x << " " << max_x << " " << min_y << " " << max_y << std::endl;


    if ( min_x == img.step+1 && max_x == -1 && min_y == img.height+1 && max_y == -1)
	return false;

    return true;
}

};

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
	
    ProcessImage processImage;

    // Handle ROS communication events
    ros::spin();

    return 0;
}
