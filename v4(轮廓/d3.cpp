#include<iostream>
#include<opencv2/opencv.hpp>
#include<vector>

int main()
{
    cv::Mat img = cv::imread("src/zhun.png");
    if(img.empty()){std::cout << "图片打开失败" << std::endl; return -1;}

    cv::Mat imgBlur;
    GaussianBlur(img, imgBlur, cv::Size(3, 3), 0);

    cv::Mat ImgHsv;
    cv::cvtColor(imgBlur, ImgHsv, cv::COLOR_BGR2HSV);

    std::vector<cv::Mat> hsvChannels;
    cv::split(ImgHsv, hsvChannels);

    cv::Scalar lower_white(0, 0, 190);
    cv::Scalar upper_white(160, 30, 255);

    cv::Mat mask;
    inRange(ImgHsv, lower_white, upper_white, mask);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    cv::Mat GauImg;
    GaussianBlur(mask, GauImg, cv::Size(7, 7), 15);
    threshold(GauImg, mask, 200, 255, cv::THRESH_BINARY);

    cv::Mat result;
    cv::Mat kernel_n = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9, 9));
    morphologyEx(mask, result, cv::MORPH_CLOSE, kernel_n);
    morphologyEx(result, result, cv::MORPH_OPEN, kernel_n);

    imshow("结果", result);

    int key = cv::waitKey(0);
    cv::destroyAllWindows();
    return 0;
}