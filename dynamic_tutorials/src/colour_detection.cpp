#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <dynamic_reconfigure/server.h>
#include <dynamic_tutorials/TutorialsConfig.h>

cv::Scalar lower = cv::Scalar(0, 0, 0);
cv::Scalar upper = cv::Scalar(180, 255, 255);

void callback(dynamic_tutorials::TutorialsConfig &config, uint32_t level) {
    lower = cv::Scalar(config.h_lower, config.s_lower, config.v_lower);
    upper = cv::Scalar(config.h_upper, config.s_upper, config.v_upper);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "dynamic_tutorials");
    ROS_INFO("Camera is originally set to detect dark blue!");
    ROS_INFO("Please run the following command to launch GUI: rosrun rqt_gui rqt_gui -s reconfigure");

    dynamic_reconfigure::Server<dynamic_tutorials::TutorialsConfig> server;
    dynamic_reconfigure::Server<dynamic_tutorials::TutorialsConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        return -1;
    }

    long area_threshold = 1000;
    cv::Mat frame, hsv, blur, mask;

    ros::Rate loop_rate(50);

    while (ros::ok()) {

        // for debugging
        // std::cout << lower << std::endl;
        // std::cout << upper << std::endl;

        // read the video frame
        cap >> frame;

        // convert the frame from bgr to hsv
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        // blur tiny pixels to the surrounding pixels
        cv::GaussianBlur(hsv, blur, cv::Size(5, 5), 0, 0);

        // get pixels within pixel ranges
        cv::inRange(blur, lower, upper, mask);

        // find the largest contour
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
        long largest_area = 0;
        long largest_contour_index = 0;

        // check for existing of contors before finding the largest
        if (contours.size()) {

            // get the largest contour based on area
            for(long i = 0; i< contours.size(); i++) {
                long area = cv::contourArea(contours[i], false);  
                if (area > largest_area) {
                    largest_area = area;
                    largest_contour_index = i;      
                }
            }

            // draw the largest contour if the contour area is greater than threshold
            if (largest_area > area_threshold) {
                cv::Scalar color = cv::Scalar(0, 255, 0);
                cv::drawContours(frame, contours, largest_contour_index, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
            }
        }

        // draw the frame with the contour and the mask
        cv::imshow("frame", frame);
        cv::imshow("mask", mask);

        // exit when ESC is pressed
        if (cv::waitKey(30) >= 0) {
            break;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}

