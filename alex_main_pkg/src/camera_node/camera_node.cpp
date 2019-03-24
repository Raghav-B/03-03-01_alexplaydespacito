#include "ros/ros.h"
#include "alex_main_pkg/camera.h"

bool take_photo(alex_main_pkg::camera::Request &req, alex_main_pkg::camera::Response &res) {
    res.output = "Photo has been taken";
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "camera_node");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("take_photo", take_photo);
    ROS_INFO("camera node active");
    ros::spin();

    return 0;
}