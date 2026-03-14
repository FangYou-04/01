#include <opencv4/opencv2/opencv.hpp>
#include <iostream>

int main()
{
    cv::Mat img = cv::imread("src/zaodian.png");
    cv::Mat img1 = cv::imread("src/zaodian2.png");
    if (img.empty()) { std::cout << "无法打开图片\n"; return -1; }
    if (img1.empty()) { std::cout << "无法打开图片\n"; return -1; }

    cv::Mat imgGray, img1Gray;
    cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(img1, img1Gray, cv::COLOR_BGR2GRAY);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5));

    cv::Mat binary ; 
    cv::threshold(imgGray, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    
    cv::Mat opening;
    cv::morphologyEx(binary, opening, cv::MORPH_OPEN, kernel);

    cv::Mat closing;
    cv::morphologyEx(opening, closing, cv::MORPH_CLOSE, kernel);

    cv::imshow("去噪1", closing);
    
    
    cv::Mat binary1 ;
    cv::threshold(img1Gray, binary1, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);

    cv::Mat closing1;
    cv::morphologyEx(binary1, closing1, cv::MORPH_CLOSE, kernel);

    cv::Mat opening1;
    cv::morphologyEx(closing1, opening1, cv::MORPH_OPEN, kernel);

    cv::Mat binary2;
    cv::threshold(opening1, binary2, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    cv::imshow("去噪2", binary2);


    int key = cv::waitKey(0);
    cv::destroyAllWindows();
    return 0;
}