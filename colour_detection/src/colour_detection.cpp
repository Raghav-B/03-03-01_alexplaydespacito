#include<stdio.h>
#include<vector>
#include<opencv2/opencv.hpp>

int main() {
    long area_threshold = 1000;
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        return -1;
    } else {
        cv::Mat frame, hsv, blur, mask;
        cv::namedWindow("frame", 1);
        while(1) {
            cap >> frame;
            cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
            cv::GaussianBlur(hsv, blur, cv::Size(5, 5), 0, 0);
            cv::inRange(blur, cv::Scalar(100, 60, 60), cv::Scalar(110, 255, 255), mask);
            std::vector<std::vector<cv::Point>> contours;
            std::vector<cv::Vec4i> hierarchy;
            cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
            long largest_area = 0;
            long largest_contour_index = 0;
            if (contours.size()) {
                for(long i = 0; i< contours.size(); i++) {
                    long area = cv::contourArea(contours[i], false);  
                    if (area > largest_area) {
                        largest_area = area;
                        largest_contour_index = i;      
                    }

                }
                if (largest_area > area_threshold) {
                    cv::Scalar color = cv::Scalar(0, 255, 0);
                    cv::drawContours(frame, contours, largest_contour_index, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
                }
            }
            cv::imshow("frame", frame);
            cv::imshow("mask", mask);
            if (cv::waitKey(30) >= 0) {
                break;
            }

        }
    }
    return 0;
}
