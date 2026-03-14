#include <opencv4/opencv2/opencv.hpp>
#include <iostream>

int main()
{
    cv::Mat img = cv::imread("src/moyin.png");
    if (img.empty()) {std::cout << "图片无法打开!" << std::endl; return -1;}

    cv::Mat imgResize;
    cv::resize(img, imgResize, cv::Size(310,465),0.5,0.5, cv::INTER_AREA); 

    cv::Mat imgMean, imgGaus, imgMedia;
    
    cv::blur(imgResize, imgMean, cv::Size(5,5));
    cv::GaussianBlur(imgResize, imgGaus, cv::Size(5,5),1);
    cv::medianBlur(imgResize, imgMedia, 5);

    cv::imshow("原图", imgResize);
    cv::imshow("均值", imgMean);
    cv::imshow("高斯", imgGaus);
    cv::imshow("中值", imgMedia);

    int key = cv::waitKey(0);
    cv::destroyAllWindows();
    return 0;
}