#include <opencv4/opencv2/opencv.hpp>
#include <iostream>
int main() 
{
   cv::Mat img = cv::imread("test.jpg");
   if (img.empty()) { std::cout << "无法打开图片\n"; return -1; }

   cv::Mat img_resized;
   resize(img, img_resized, cv::Size(400, 300), 0, 0, cv::INTER_AREA);

   cv::imshow("显示窗口", img);
   cv::imshow("缩小后", img_resized);
   cv::waitKey(0);
   return 0;
}