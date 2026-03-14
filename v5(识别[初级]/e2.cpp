// #include <opencv4/opencv2/opencv.hpp>
// #include <iostream>

// int hmin = 0, smin = 0, vmin = 0;
// int hmax = 179, smax = 255, vmax = 255;
// int main() 
// {
// 	cv::Mat img = cv::imread("src/3(hong.jpg");
// 	cv::Mat imgResize, imgHSV, mask;
//     cv::resize(img, imgResize, cv::Size(800,800));
// 	cv::cvtColor(imgResize, imgHSV, cv::COLOR_BGR2HSV);
// 	cv::namedWindow("Trackbars", (640, 200));

// 	cv::createTrackbar("Hue Min", "Trackbars", &hmin, 179);
// 	cv::createTrackbar("Hue Max", "Trackbars", &hmax, 179);
// 	cv::createTrackbar("Sat Min", "Trackbars", &smin, 255);
// 	cv::createTrackbar("Sat Max", "Trackbars", &smax, 255);
// 	cv::createTrackbar("Val Min", "Trackbars", &vmin, 255);
// 	cv::createTrackbar("Val Max", "Trackbars", &vmax, 255);
	
// 	while (true) {
// 		cv::Scalar lower(hmin, smin, vmin);
// 		cv::Scalar upper(hmax, smax, vmax);
// 		cv::inRange(imgHSV, lower, upper, mask);
// 		cv::imshow("Image", imgResize);
// 		cv::imshow("Image HSV", imgHSV);
// 		cv::imshow("Image mask", mask);
// 		cv::waitKey(1);
// 	}
// }

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

// 全局变量：存储HSV阈值和图像
Mat src, hsv, mask, mask1;
int h_min1 = 0, h_max1 = 10;
int s_min1 = 43, s_max1 = 255;
int v_min1 = 46, v_max1 = 255;
int h_min2 = 156, h_max2 = 180;
int s_min2 = 43, s_max2 = 255;
int v_min2 = 46, v_max2 = 255;


// 滑动条回调函数：更新掩码
void updateMask(int, void*) {
    // Scalar lower = Scalar(h_min1, s_min1, v_min1);
    // Scalar upper = Scalar(h_max1, s_max1, v_max1);
    Scalar lower1 = Scalar(h_min2, s_min2, v_min2);
    Scalar upper1 = Scalar(h_max2, s_max2, v_max2);
    // inRange(hsv, lower, upper, mask);
    inRange(hsv, lower1, upper1, mask1);
    // 形态学优化（和之前提取文字的逻辑一致）s"); // 替换为你
    Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    // morphologyEx(mask, mask, MORPH_CLOSE, kernel);
    // morphologyEx(mask, mask, MORPH_OPEN, kernel);
    morphologyEx(mask1, mask1, MORPH_CLOSE, kernel);
    morphologyEx(mask1, mask1, MORPH_OPEN, kernel);

    // imshow("HSV调节掩码", mask);
    imshow("HSV调节掩码2", mask1);
}

int main() {
    // 1. 读取图像并转换颜色空间
    src = imread("src/3(hong.jpg"); // 替换为你的图片路径
    if (src.empty()) {
        cout << "无法读取图像！" << endl;
        return -1;
    }
    cvtColor(src, hsv, COLOR_BGR2HSV);

    // 2. 创建窗口并添加滑动条
    namedWindow("HSV调节面板", WINDOW_NORMAL);
    // Hue通道（0-180）
    createTrackbar("H最小值", "HSV调节面板", &h_min2, 180, updateMask);
    createTrackbar("H最大值", "HSV调节面板", &h_max2, 180, updateMask);
    // Saturation通道（0-255）
    createTrackbar("S最小值", "HSV调节面板", &s_min2, 255, updateMask);
    createTrackbar("S最大值", "HSV调节面板", &s_max2, 255, updateMask);
    // Value通道（0-255）
    createTrackbar("V最小值", "HSV调节面板", &v_min2, 255, updateMask);
    createTrackbar("V最大值", "HSV调节面板", &v_max2, 255, updateMask);

    // // 2. 创建窗口并添加滑动条
    // namedWindow("HSV调节面板2", WINDOW_NORMAL);
    // // Hue通道（0-180）
    // createTrackbar("H最小值", "HSV调节面板", &h_min2, 180, updateMask);
    // createTrackbar("H最大值", "HSV调节面板", &h_max2, 180, updateMask);
    // // Saturation通道（0-255）
    // createTrackbar("S最小值", "HSV调节面板", &s_min2, 255, updateMask);
    // createTrackbar("S最大值", "HSV调节面板", &s_max2, 255, updateMask);
    // // Value通道（0-255）
    // createTrackbar("V最小值", "HSV调节面板", &v_min2, 255, updateMask);
    // createTrackbar("V最大值", "HSV调节面板", &v_max2, 255, updateMask);

    // 3. 初始化掩码并显示窗口

    // 4. 等待按键，按下ESC退出并输出当前阈值
    while (true) {
        int key = waitKey(10);
        if (key == 27) { // ESC键
            cout << "最优HSV阈值：" << endl;
            cout << "H范围：[" << h_min2 << ", " << h_max2 << "]" << endl;
            cout << "S范围：[" << s_min2 << ", " << s_max2 << "]" << endl;
            cout << "V范围：[" << v_min2 << ", " << v_max2 << "]" << endl;
            // cout << "最优HSV阈值：" << endl;
            // cout << "H范围：[" << h_min2 << ", " << h_max2 << "]" << endl;
            // cout << "S范围：[" << s_min2 << ", " << s_max2 << "]" << endl;
            // cout << "V范围：[" << v_min2 << ", " << v_max2 << "]" << endl;
            break;
        }
    }

    destroyAllWindows();
    return 0;
}