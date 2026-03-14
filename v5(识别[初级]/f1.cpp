#include <opencv4/opencv2/opencv.hpp>
#include <iostream>

cv::Mat turnHSV(const cv::Mat & img)
{
    cv::Mat imgGaus, imgHSV;
    GaussianBlur(img, imgGaus, cv::Size(3, 3), 2);
    cv::cvtColor(imgGaus, imgHSV, cv::COLOR_BGR2HSV);

    cv::Scalar lower_red1 = cv::Scalar(0,43,46);
    cv::Scalar upper_red1 = cv::Scalar(0,255,255);
    cv::Scalar lower_red2 = cv::Scalar(156,43,46);
    cv::Scalar upper_red2 = cv::Scalar(180,255,255);
    

    cv::Mat mask1, mask2, maskRED;
    cv::inRange(imgHSV, lower_red1, upper_red1, mask1);
    cv::inRange(imgHSV, lower_red2, upper_red2, mask2);
    cv::bitwise_or(mask1, mask2, maskRED);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE , cv::Size(5,5));
    cv::Mat img_N;
    cv::morphologyEx(maskRED, img_N, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(img_N, img_N, cv::MORPH_OPEN, kernel);

    return img_N;
}

struct light
{
    cv::RotatedRect rect;
    float rat;
    float AbsAngle;
    cv::Point2f center;
};


std::vector<light> FindLightContours(const cv::Mat & img)
{
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<light> Edge;
    for (size_t i = 0; i < contours.size(); i++)
    {
        double area = cv::contourArea(contours[i]);
        if (area < 100)
        {
            continue;
        }
        
        cv::RotatedRect rRect = cv::minAreaRect(contours[i]);
        float width = rRect.size.width;
        float height = rRect.size.height;
        if(width > height) cv::swap(width, height);

        float ratio = (width == 0) ? 0 :height / width;
        float angle = fabs(rRect.angle);

        if(ratio > 2 && ratio < 5 && angle < 15)
        {
            light lb;
            lb.rect = rRect;
            lb.rat = ratio;
            lb.AbsAngle = angle;
            lb.center = rRect.center;
            Edge.push_back(lb);
        }
    }
    return Edge;
}

struct Armor
{
    light left;
    light right;
    cv::Rect boundingRect;
    cv::Point2f center;
};

std::vector<Armor> FindArmorContours(const std::vector<light> & Edge)
{
    std::vector<Armor> armors;
    if (Edge.size() < 2) return armors; 

    for (size_t i = 0; i < Edge.size(); i++)
    {           
        for (size_t j = i + 1 ; j < Edge.size(); j++)
        {
            const light &bar1 = Edge[i];
            const light &bar2 = Edge[j];

            const light &leftbar =bar1.center.x < bar2.center.x ? bar1 : bar2;
            const light &rightbar =bar1.center.x < bar2.center.x ? bar2 : bar1;

            float distance = cv::norm(leftbar.center - rightbar.center);
            float highAvege = (leftbar.rect.size.height + rightbar.rect.size.height)/2;

            if (distance < highAvege || distance > 2 * highAvege)
            {
                continue;
            }
            
            float angle = fabs(atan2(rightbar.center.y - leftbar.center.y,
                     rightbar.center.x -leftbar.center.x) * 180/ CV_PI);
            
            if (angle > 20)
            {
                continue;
            }
            
            float hightDiff = fabs(leftbar.rect.size.height - rightbar.rect.size.height)
                    / std::max(leftbar.rect.size.height, rightbar.rect.size.height);

            if (hightDiff > 0.5)
            {
                continue;
            }
            
            Armor armor;
            armor.left = leftbar;
            armor.right = rightbar;

            cv::Rect rect1 = leftbar.rect.boundingRect();
            cv::Rect rect2 = rightbar.rect.boundingRect();

            armor.boundingRect = cv::Rect(
                std::min(rect1.x, rect2.x),
                std::min(rect1.y, rect2.y),
                std::max(rect1.x + rect1.width, rect2.x + 
                    rect2.width) - std::min(rect1.x, rect2.x),
                std::min(rect1.y + rect1.height, rect2.y +
                    rect2.height) - std::min(rect1.y, rect2.y)
            );
            armor.center = (leftbar.center + rightbar.center) * 0.5f;
            armors.push_back(armor);
        }
    }
    return armors;
}

void drawcontours(cv::Mat &img, std::vector<light> Edge, std::vector<Armor> armors)
{
    for(const auto &l : Edge)
    {
        cv::Point2f pts[4];
        l.rect.points(pts);
        for (int k = 0; k < 4; k++)
        {
            cv::line(img, pts[k], pts[(k+1)%4], cv::Scalar(0,255,0), 2);
        }
        cv::circle(img, l.center, 3, cv::Scalar(255,0,0),-1);
    }
}

int main()
{
    cv::Mat img = cv::imread("src/3(hong.jpg");
    if (img.empty()) {std::cout << "图片打开失败！"; return -1;}

    cv::Mat img_n = img.clone();
    cv::Mat mask = turnHSV(img);

    std::vector<light> Edge = FindLightContours(mask);
    std::vector<Armor> armors = FindArmorContours(Edge);

    drawcontours(img_n, Edge, armors);

    cv::imshow("原图", img);
    cv::imshow("红", mask);
    cv::imshow("装甲板", img_n);

    int key = cv::waitKey(0);
    cv::destroyAllWindows();
    return 0;
}