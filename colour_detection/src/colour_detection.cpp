#include<stdio.h>
#include<vector>
#include<opencv2/opencv.hpp>

int main() {
    // red hsv values
    cv::Scalar lower_red = cv::Scalar(100, 60, 60);
    cv::Scalar higher_red = cv::Scalar(110, 255, 255);

    // blue hsv values
    cv::Scalar lower_blue = cv::Scalar(100, 60, 60);
    cv::Scalar higher_blue = cv::Scalar(100, 60, 60);

    // area has to greater than the threshold to be detected 
    long area_threshold = 1000;

    // initialized the camaera
    cv::VideoCapture cap(0);

    // check whether the camera is successfully initialized
    if (!cap.isOpened()) {

        return -1;

    } else {

        // frames are stored in type MAT
        cv::Mat frame, hsv, blur, mask;

        while(1) {

            // read the video frame
            cap >> frame;

            // convert the frame from bgr to hsv
            cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

            // blur tiny pixels to the surrounding pixels
            cv::GaussianBlur(hsv, blur, cv::Size(5, 5), 0, 0);

            // get pixels within pixel ranges
            cv::inRange(blur, lower_red, higher_red, mask);

            // find the largest contour
            std::vector<std::vector<cv::Point>> contours;
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

        }
    }
    return 0;
}
