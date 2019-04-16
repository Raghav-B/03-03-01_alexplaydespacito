#include<stdio.h>
#include<vector>
#include<opencv2/opencv.hpp>
#include<string>
#include<iostream>

using namespace std;
int main() {

    // red hsv values
    cv::Scalar lower_red = cv::Scalar(170, 140, 140);
    cv::Scalar higher_red = cv::Scalar(180, 180, 220);

    // green hsv values
    cv::Scalar lower_green = cv::Scalar(40, 70, 100);
    cv::Scalar higher_green = cv::Scalar(70, 140, 170);

    // area has to greater than the threshold to be detected 
    long area_threshold = 1000;

    cv::VideoCapture cap(0);

    if (!cap.isOpened()) {
        return -1;

    } else {

        while (true) {

            // frames are stored in type MAT
            cv::Mat frame, hsv, blur, mask;


            while (true) {

                // read the image frame
                cap >> frame;

                // convert the frame from bgr to hsv
                cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

                // blur tiny pixels to the surrounding pixels
                cv::GaussianBlur(hsv, blur, cv::Size(5, 5), 0, 0);

                // flag to check for presence of color
                bool green_found = false;

                // find the largest contour
                std::vector<std::vector<cv::Point> > contours;
                std::vector<cv::Vec4i> hierarchy;

                // get pixels within pixel ranges
                cv::inRange(blur, lower_green, higher_green, mask);

                // find the largest contour
                cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
                long largest_area_green = 0;
                long largest_contour_green_index = 0;

                // moments
                cv::Moments m;

                // centroid x y coordinates
                cv::Point2f c;

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
                        m = cv::moments(contours[largest_contour_green_index], false);
                        c = cv::Point2f( m.m10/m.m00 , m.m01/m.m00 );
                    }
                }

                // draw the frame with the contour and the mask for debugging
                cv::imshow("frame", frame);
                if (cv::waitKey(30) >= 0) {
                    break;
                }

                if (green_found) {
                    std::cout << "Green found" << std::endl;
                    int left_bar = frame.cols * 2 / 3;
                    int right_bar = frame.cols / 3;
                    if (c.x > left_bar) {
                        std::cout << "object to left" << std::endl;
                    } else if (c.x > right_bar) {
                        std::cout << "object to middle" << std::endl;
                    } else {
                        std::cout << "object to right" << std::endl;
                    }
                    if (largest_area_green < 5000) {
                        std::cout << "object is far" << std::endl;
                    } else {
                        std::cout << "object is close" << std::endl;
                    }
                    std::cout << largest_area_green << std::endl;
                } else {
                    std::cout << "Green not found" << std::endl;
                }
            }
        }

    }

    return 0;
}
