#include "ros/ros.h"
#include "alex_main_pkg/camera.h"
#include<stdio.h>
#include<vector>
#include<opencv2/opencv.hpp>
#include<string>
#include<iostream>

// red hsv values
cv::Scalar lower_red = cv::Scalar(170, 140, 140);
cv::Scalar higher_red = cv::Scalar(180, 180, 220);

// green hsv values
cv::Scalar lower_green = cv::Scalar(40, 70, 100);
cv::Scalar higher_green = cv::Scalar(70, 140, 170);

// area has to greater than the threshold to be detected 
long area_threshold = 1000;

bool take_photo(alex_main_pkg::camera::Request &req, alex_main_pkg::camera::Response &res) {
    std::system("raspistill -o photo.jpg");

    // image name
    std::string imageName("photo.jpg"); 

    // frames are stored in type MAT
    cv::Mat frame, hsv, blur, mask;

    // read the image frame
    frame = cv::imread(imageName, cv::IMREAD_COLOR);

    // convert the frame from bgr to hsv
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    // blur tiny pixels to the surrounding pixels
    cv::GaussianBlur(hsv, blur, cv::Size(5, 5), 0, 0);

    // flag to check for presence of color
    bool red_found = false, green_found = false;

    // get pixels within pixel ranges
    cv::inRange(blur, lower_red, higher_red, mask);

    // find the largest contour
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
    long largest_area_red = 0;
    long largest_contour_red_index = 0;

    // check for existing of contors before finding the largest
    if (contours.size()) {

        // get the largest contour based on area
        for(long i = 0; i< contours.size(); i++) {
            long area = cv::contourArea(contours[i], false);  
            if (area > largest_area_red) {
                largest_area_red = area;
                largest_contour_red_index = i;      
            }

        }

        // draw the largest contour if the contour area is greater than threshold
        if (largest_area_red > area_threshold) {
            cv::Scalar color = cv::Scalar(0, 255, 0);
            cv::drawContours(frame, contours, largest_contour_red_index, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
            red_found = true;
        }
    }

    // get pixels within pixel ranges
    cv::inRange(blur, lower_green, higher_green, mask);

    // find the largest contour
    cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
    long largest_area_green = 0;
    long largest_contour_green_index = 0;

    // check for existing of contors before finding the largest
    if (contours.size()) {

        // get the largest contour based on area
        for(long i = 0; i< contours.size(); i++) {
            long area = cv::contourArea(contours[i], false);  
            if (area > largest_area_green) {
                largest_area_green = area;
                largest_contour_green_index = i;      
            }

        }

        // draw the largest contour if the contour area is greater than threshold
        if (largest_area_green > area_threshold) {
            cv::Scalar color = cv::Scalar(0, 255, 0);
            cv::drawContours(frame, contours, largest_contour_green_index, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
            green_found = true;
        }
    }

    if (req.input == "start detection") {
        if (green_found && red_found) {
            res.output = "Red and Green Detected";
        } else if (green_found) {
            res.output = "Green Detected";
        } else if (red_found) {
            res.output = "Red Detected";
        } else {
            res.output = "No Color Detected";
        }
    } else {
        res.output = "Detection Failed";
    }
    
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "camera_node");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("take_photo", take_photo);
    ROS_INFO("Camera Node Started");
    
    ros::spin();

    return 0;
}


