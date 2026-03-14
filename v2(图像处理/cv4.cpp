#include <opencv4/opencv2/opencv.hpp>
#include <iostream>
#include <vector>

int main()
{
    cv::Mat img = cv::imread("src/3.jpg");
    if (img.empty()) { std::cout << "无法打开图片\n"; return -1; }
    cv::Mat imgGray;
    cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

    std::vector <cv::Mat> bgr;
    cv::split(img, bgr);

    cv::imshow("红", bgr[2]);
    cv::imshow("绿", bgr[1]);
    cv::imshow("蓝", bgr[0]);
    cv::imshow("灰度", imgGray);

    int key = cv::waitKey(0);
    cv::destroyAllWindows();
    return 0;
}