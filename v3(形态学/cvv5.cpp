#include <opencv4/opencv2/opencv.hpp>
#include <iostream>

int main()
{
    cv::Mat img1 = cv::imread("src/origin2.png");
    cv::Mat img2 = cv::imread("src/blue.png");
    if (img1.empty()) { std::cout << "无法打开图片\n"; return -1; }
    if (img2.empty()) { std::cout << "无法打开图片\n"; return -1; }

    cv::Mat img1Resize, img2Resize;
    cv::resize(img1, img1Resize, cv::Size(500,500));
    cv::resize(img2, img2Resize, cv::Size(500,500));

    std::vector<cv::Mat> BGRarray1, BGRarray2;
    cv::split(img1Resize, BGRarray1);
    cv::split(img2Resize, BGRarray2);

    cv::Mat img1Zero = cv::Mat::zeros(img1Resize.size(), CV_8UC1);
    
    cv::Mat img2Zero = cv::Mat::zeros(img2Resize.size(), CV_8UC1);

    std::vector<cv::Mat> merge_all;

    merge_all = {BGRarray1[0], img1Zero, img1Zero};
    cv::Mat img1B;
    cv::merge(merge_all, img1B);

    merge_all = {img1Zero, BGRarray1[1], img1Zero};
    cv::Mat img1G;
    cv::merge(merge_all, img1G);

    merge_all = {img1Zero, img1Zero, BGRarray1[2]};
    cv::Mat img1R;
    cv::merge(merge_all, img1R);

    merge_all = {BGRarray2[0], img2Zero, img2Zero};
    cv::Mat img2B;
    cv::merge(merge_all, img2B);

    merge_all = {img2Zero, BGRarray2[1], img2Zero};
    cv::Mat img2G;
    cv::merge(merge_all, img2G);

    merge_all = {img2Zero, img2Zero, BGRarray2[2]};
    cv::Mat img2R;
    cv::merge(merge_all, img2R);

    cv::imshow("原图-1", img1Resize);
    cv::imshow("R通道-1", img1R);
    cv::imshow("G通道-1", img1G);
    cv::imshow("B通道-1", img1B);

    cv::imshow("原图-2", img2Resize);
    cv::imshow("R通道-2", img2R);
    cv::imshow("G通道-2", img2G);
    cv::imshow("B通道-2", img2B);

    int key = cv::waitKey(0);
    cv::destroyAllWindows();
    return 0;
}