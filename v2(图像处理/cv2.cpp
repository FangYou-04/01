#include <opencv4/opencv2/opencv.hpp>
#include <iostream>
int main()
{
    cv::VideoCapture cap ("src/3.mp4");
    if (!cap.isOpened())
    {
        std::cout << "无法打开视频" << std::endl;
        return -1;
    }
    cv::Mat frame;
    cv::namedWindow("视频", cv::WINDOW_NORMAL);
    while (true)
    {
        if (!cap.read(frame))
        {
            std::cout << "视频播放完毕" << std::endl;
            break;
        }
        if (frame.empty())
        {
            std::cout << "空帧" << std::endl;
            continue;
        }
        
        cv::imshow("视频", frame);
        int key = cv::waitKey(50);
        if (key == 'Q' || key == 'q')  //单引号是char 双引号是string 
        {
            std::cout << "按q退出" << std::endl;
            cv::destroyAllWindows();
            break;
        }
    }
    cap.release();
    return 0;
}