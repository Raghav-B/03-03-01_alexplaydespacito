#include "ros/ros.h"
#include "alex_main_pkg/camera.h"

void take_photo(alex_main_pkg::camera::Request &input, alex_main_pkg::camera::Response &output) {
    output = "Photo has been taken";
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "camera_node");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("take_photo", take_photo);
    ROS_INFO("camera_node active");
    ros::spin();

    return 0;
}