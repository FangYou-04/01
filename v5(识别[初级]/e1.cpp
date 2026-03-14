#include <opencv4/opencv2/opencv.hpp>
#include <iostream>

int main()
{
    cv::Mat img =cv::imread("src/3(hong.jpg");
    if(img.empty()){std::cout << "图片无法打开！"; return -1;}

    cv::Mat imgResize;
    cv::resize(img, imgResize, cv::Size(500,500));
    
    cv::Mat imgGray;
    cv::cvtColor(imgResize, imgGray, cv::COLOR_BGR2GRAY);
    cv::imshow("灰度", imgGray);

    cv::Mat binary;
    cv::threshold(imgGray, binary, 120, 255, cv::THRESH_BINARY);
    cv::imshow("二值", binary);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    
    cv::Mat drawcontours = imgResize.clone();
    // for (const auto & contour:contours)
    // {
    //     tools::draw_point(drawcontours,contour);
    // }
    for (int i = 0; i < contours.size(); i++) 
    {
        cv::drawContours(drawcontours, contours, i, cv::Scalar(0, 255, 0), 1);
    }
    cv::imshow("轮廓", drawcontours);



    int key = cv::waitKey(0);
    cv::destroyAllWindows();
    return 0;
}