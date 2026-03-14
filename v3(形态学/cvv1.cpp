#include <opencv4/opencv2/opencv.hpp>
#include <iostream>

int main()
{
    cv::VideoCapture cap(0);
    if (!cap.isOpened())
    {
        std::cout << "无法打开摄像头！" << std::endl;
        return -1;
    }
    
    cv::Mat img;

    while (true)
    {
        cap.read(img);

        cv::imshow("摄像头", img);
        cv::waitKey(1);     
        
        int key = cv::waitKey(50);
        if (key == 'q')
        {
            cv::destroyAllWindows;
            break;
        }     
    }
    cap.release();
    return 0;
}