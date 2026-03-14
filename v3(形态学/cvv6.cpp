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

    cv::Mat B1 = BGRarray1[0], R1 = BGRarray1[2];
    cv::Mat B2 = BGRarray2[0], R2 = BGRarray2[2];

    cv::Mat B1_R1, R1_B1, B1_R1_N, R1_B1_N;
    cv::subtract(B1, R1, B1_R1, cv::noArray(), CV_16S);
    cv::subtract(R1, B1, R1_B1, cv::noArray(), CV_16S);
    cv::normalize(B1_R1, B1_R1_N, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::normalize(R1_B1, R1_B1_N, 0, 255, cv::NORM_MINMAX, CV_8UC1);

    cv::Mat B2_R2, R2_B2, B2_R2_N, R2_B2_N;
    cv::subtract(B2, R2, B2_R2, cv::noArray(), CV_16S);
    cv::subtract(R2, B2, R2_B2, cv::noArray(), CV_16S);
    cv::normalize(B2_R2, B2_R2_N, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::normalize(R2_B2, R2_B2_N, 0, 255, cv::NORM_MINMAX, CV_8UC1);

    cv::Mat img1Zero = cv::Mat::zeros(img1Resize.size(), CV_8UC1);
    
    cv::Mat img2Zero = cv::Mat::zeros(img2Resize.size(), CV_8UC1);

    std::vector<cv::Mat> merge_all;

    merge_all = {B1_R1_N, BGRarray1[1], img1Zero};
    cv::Mat img1B1_R1;
    cv::merge(merge_all, img1B1_R1);

    merge_all = {img1Zero, BGRarray1[1], R1_B1_N};
    cv::Mat img1R1_B1;
    cv::merge(merge_all, img1R1_B1);

    merge_all = {B2_R2_N, BGRarray2[1], img2Zero};
    cv::Mat img2B2_R2;
    cv::merge(merge_all, img2B2_R2);

    merge_all = {img2Zero, BGRarray2[1], R2_B2_N};
    cv::Mat img2R2_B2;
    cv::merge(merge_all, img2R2_B2);

    cv::imshow("B1-R1", img1B1_R1);
    cv::imshow("R1_B1", img1R1_B1);
    cv::imshow("B2-R2", img2B2_R2);
    cv::imshow("R2_B2", img2R2_B2);

    int key = cv::waitKey(0);
    cv::destroyAllWindows();
    return 0;
}