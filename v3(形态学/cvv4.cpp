#include <opencv4/opencv2/opencv.hpp>
#include <iostream>

int main()
{
    cv::Mat img =cv::imread("src/origin.png");
    if (img.empty()) {std::cout << "图片打开失败\n"; return -1;}

    cv::Mat imgResize;
    cv::resize(img, imgResize, cv::Size(500,500));
    cv::imshow("原图", imgResize);

    cv::Mat hsv;
    cv::cvtColor(imgResize, hsv, cv::COLOR_BGR2HSV);

    cv::Scalar red1min(0, 43, 46);
    cv::Scalar red1max(10, 255, 255);
    cv::Scalar red2min(156, 43, 46);
    cv::Scalar red2max(180, 255, 255);

    cv::Mat mask_1, mask_2, mask;
    cv::inRange(hsv, red1min, red1max, mask_1);
    cv::inRange(hsv, red2min, red2max, mask_2);
    mask = mask_1 + mask_2;

    cv::Mat imgBlue = imgResize.clone();
    imgBlue.setTo(cv::Scalar(225,0,0), mask);
    cv::imshow("蓝色", imgBlue);

    cv::Mat imgblue = imgBlue.clone();

    cv::Mat imgGray, imgEdges;
    cv::cvtColor(imgResize, imgGray, cv::COLOR_BGR2GRAY);

    cv::Mat imgGaus;
    cv::GaussianBlur(imgGray, imgGaus, cv::Size(5,5), 1);

    cv::Canny(imgGaus, imgEdges, 30, 100, 3);
    cv::imshow("边缘", imgEdges);

    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(imgEdges, circles, cv::HOUGH_GRADIENT, 1,
    40, 100, 40, 5, 50);
    
    for (size_t i = 0; i < circles.size(); i++)
    {
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        cv::circle(imgblue, center, 3, cv::Scalar(255,0,0), -1);
        cv::circle(imgblue, center, radius, cv::Scalar(0,0,255), 2);
    }
    cv::imshow("霍夫变换", imgblue);
    
    int key = cv::waitKey(0);
    cv::destroyAllWindows();
    return 0;
}