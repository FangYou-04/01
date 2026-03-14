#include <opencv4/opencv2/opencv.hpp>
#include <iostream>

int main()
{
    cv::Mat img = cv::imread("src/fengche.png");
    if (img.empty()) {std::cout << "图片无法打开！"; return -1;}

    cv::Mat imghsv,imgN;
    cv::cvtColor(img, imghsv, cv::COLOR_BGR2HSV);

    cv::Scalar red1min(0, 120, 70);
    cv::Scalar red1max(10, 255, 255);
    cv::Scalar red2min(170, 120, 70);
    cv::Scalar red2max(180, 255, 255);

    cv::Mat mask_1, mask_2, mask;
    cv::inRange(imghsv, red1min, red1max, mask_1);
    cv::inRange(imghsv, red2min, red2max, mask_2);
    mask = mask_1 + mask_2;

    cv::bitwise_and(img, img, imgN, mask);

    cv::Mat imgEdge;
    cv::Canny(imgN, imgEdge, 50, 150, 3);
    
    cv::imshow("原图", img);
    cv::imshow("轮廓", imgEdge);

    int key = cv::waitKey(0);
    cv::destroyAllWindows();
    return 0;
}

// int man()
// {
//     cv::Mat img = cv::imread("./src1/opencv5.1.png");

//     cv::Mat imgHsv, imgCanny;
//     if(img.empty())
//     {
//         std::cerr <<"图片打开失败"<< std::endl;
//         return -1;
//     }
//     cv::cvtColor(img, imgHsv, cv::COLOR_BGR2HSV);

//     cv::Scalar lower_red1 = cv::Scalar(0,120,70);
//     cv::Scalar upper_red1 = cv::Scalar(10,255,255);
//     cv::Scalar lower_red2 = cv::Scalar(170,120,70);
//     cv::Scalar upper_red2 = cv::Scalar(180,255,255);

//     cv::Mat mask1, mask2, redMask;
//     inRange(imgHsv, lower_red1, upper_red1,mask1);
//     inRange( imgHsv,lower_red2, upper_red2,mask2);
//     redMask = mask1 | mask2;

//     cv::Mat imgGray;
//     cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

//     cv::Mat redGray;
//     imgGray.copyTo(redGray, redMask);
//     cv::Canny(redGray, imgCanny, 100, 200);

//     cv::imshow("原图", img);
//     cv::imshow("轮廓", imgCanny);

//     int key = cv::waitKey(0);
//     cv::destroyAllWindows();
//     return 0;
// }