#include "Config.hpp"
#include "Struct.hpp"
#include <opencv2/opencv.hpp>
#include <cmath>
#include <vector>

// 灯条配对
bool isLightMatch(const LightBar& left, const LightBar& right) 
{
    // 1. 角度差判断：灯条角度差不能超过阈值
    // 装甲板左右灯条应该保持平行，角度差很小
    if (fabs(left.angle - right.angle) > LIGHT_ANGLE_MAX) 
    {
        return false;
    }
    // 2. 长度比例判断：灯条长度比例不能相差太大
    // 装甲板左右灯条长度应该相近，避免匹配到不同大小的灯条
    float lengthRatio = std::min(left.length, right.length) / std::max(left.length, right.length);
    if (lengthRatio < (1 - LIGHT_LENGTH_DIFF_RATIO)) 
    {
        return false;
    }
    // 3. 位置判断：确保左灯条在左，右灯条在右
    // 装甲板的左右灯条应该符合视觉上的左右位置关系
    if (left.center.x >= right.center.x) 
    {
        return false; // 保证left在左边，right在右边
    }
    // 4. 连线角度判断：灯条中心连线与水平方向的夹角不能太大
    // 装甲板的左右灯条应该基本在同一水平线上
    float deltaY = right.center.y - left.center.y;
    float deltaX = right.center.x - left.center.x;
    float angle = fabs (atan2(deltaY, deltaX) * 180.0 / CV_PI);
    if (angle > ARMOR_ANGLE_MAX)
    {
        return false;
    }
    // 5. 距离判断：灯条中心距离应该在合理范围内
    // 装甲板的宽度（灯条中心距离）应该与灯条长度成比例
    float Distance = cv::norm(left.center - right.center);
    float avgLength = (left.length + right.length) / 2.0f;
    if (Distance < avgLength * DISTANCE_MIN || Distance > avgLength * DISTANCE_MAX) 
    {
        return false;
    }
    // 所有条件都满足，可以配对成装甲板
    return true;
}

// 装甲板匹配主函数
std::vector<Armor> matchArmors(const std::vector<LightBar>& lightBars) 
{
    std::vector<Armor> armors;
    if (lightBars.size() < 2)
    {
        return armors; // 不足两个灯条，无法配对
    }
    // 遍历所有灯条组合，尝试配对
    for (size_t i = 0; i < lightBars.size(); i++)
    {
        for (size_t j = 0; j < lightBars.size(); j++)
        {
            if (isLightMatch(lightBars[i], lightBars[j])) 
            {
                armors.emplace_back(lightBars[i], lightBars[j]);
            }
        }
    }
    return armors;
}



