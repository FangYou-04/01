#include <opencv4/opencv2/opencv.hpp>
#include <iostream>
int main() 
{
   cv::Mat img = cv::imread("src/1.jpg");
   cv::Mat imgg = cv::imread("src/2.jpg");
   if (img.empty()) { std::cout << "无法打开图片\n"; return -1; }
   if (imgg.empty()) { std::cout << "无法打开图片\n"; return -1; }

   cv::Mat img_resized;
   resize(img, img_resized, cv::Size(500, 800), 0, 0, cv::INTER_AREA);
   cv::Mat imgg_resized;
   resize(imgg, imgg_resized, cv::Size(500,800));

   cv::imshow("图片1", img_resized);
   cv::imshow("图片2", imgg_resized);
   int key = cv::waitKey(0);
   cv::destroyAllWindows();
   return 0;
}