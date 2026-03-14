#include <opencv4/opencv2/opencv.hpp>
#include <iostream>
int main()
{
    cv::Mat img = cv::imread("src/1.jpg");
    if (img.empty()) { std::cout << "无法打开图片\n"; return -1; }
    cv::Mat imgResize;
    cv::resize(img, imgResize, cv::Size(300, 300),0,0);

    cv::Mat imgResize_h;
    cv::flip(imgResize, imgResize_h, 1);
    cv::Mat imgResize_v;
    cv::flip(imgResize, imgResize_v, 0);

    cv::imwrite("flip_h.jpg", imgResize_h);
    cv::imwrite("flip_v.jpg", imgResize_v);

    cv::imshow("缩小", imgResize);
    cv::imshow("水平", imgResize_h);
    cv::imshow("垂直", imgResize_v);

    int key = cv::waitKey(0);
    cv::destroyAllWindows();
    return 0;
}