#include<stdio.h>
#include<opencv2/opencv.hpp>

int main() {
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        return -1;
    } else {
        cv::Mat edges;
        cv::namedWindow("frame", 1);
        for(;;) {
            cv::Mat frame;
            cap >> frame;
            cv::cvtColor(frame, edges, cv::COLOR_BGR2GRAY);
            cv::GaussianBlur(edges, edges, cv::Size(7, 7), 1.5, 1.5);
            cv::Canny(edges, edges, 0, 30, 3);
            cv::imshow("edges", edges);
            if (cv::waitKey(30) >= 0) break;
        }
    }
    return 0;
}
