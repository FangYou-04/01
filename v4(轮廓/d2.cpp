#include <opencv4/opencv2/opencv.hpp>
#include <iostream>
#include <algorithm>
#include <tuple>

int main()
{
    cv::Mat img = cv::imread("src/xing.png");
    if(img.empty()){std::cout << "图片无法打开！"; return -1;}

    cv::Mat imgGRAY, imgGAUS, imgCanny, imgDilate;

    cv::cvtColor(img, imgGRAY, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(imgGRAY, imgGAUS, cv::Size(5,5), 1);
    cv::Canny(imgGAUS, imgCanny, 50, 150, 3);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
    cv::dilate(imgCanny, imgDilate, kernel);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(imgDilate, contours, hierarchy, cv::RETR_EXTERNAL, 
        cv::CHAIN_APPROX_SIMPLE);

    std::vector<std::tuple<double, double, std::vector<cv::Point>>> contourData;

    for (int i = 0; i < contours.size(); i++)
    {
        double area = cv::contourArea(contours[i]);
        if (area < 10) continue;

        cv::Rect out = cv::boundingRect(contours[i]);
        double rat = static_cast<float>(out.width)/out.height;

        contourData.emplace_back(area, rat, contours[i]);
    }

    if (contourData.empty()) {
        std::cout << "未检测到有效轮廓！" << std::endl;
        return -1;
    }

    std::sort(contourData.begin(), contourData.end(),
          [](const auto& a, const auto& b) {
              return std::get<0>(a) < std::get<0>(b); 
          });

    auto minContour = contourData.front();  
    auto maxContour = contourData.back();   
    
    std::cout << "最小轮廓面积: " << std::get<0>(minContour) 
              << ", 宽高比: " << std::fixed << std::setprecision(2) <<
               std::get<1>(minContour) << std::endl;
    std::cout << "最大轮廓面积: " << std::get<0>(maxContour) 
              << ", 宽高比: " << std::fixed << std::setprecision(2) <<
               std::get<1>(maxContour) << std::endl;

    cv::RotatedRect minRect = cv::minAreaRect(std::get<2>(maxContour));
    cv::Point2f vertices[4];
    minRect.points(vertices);
    for (size_t i = 0; i < 4; i++) {
        cv::line(img, vertices[i], vertices[(i+1)%4], cv::Scalar(0, 255, 0), 2);
    }
    
    cv::imshow("Result", img);

    int key = cv::waitKey(0);
    cv::destroyAllWindows();
    return 0;
}