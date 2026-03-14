#ifndef CONFIG_H
#define CONFIG_H



// 高斯模糊参数
#define GAUSSIAN_SIZE 7
#define GAUSSIAN_SIGMA 0

// 掩码参数（OpenCV HSV: H:0-179, S:0-255, V:0-255）
// 红色在 HSV 空间通常分为两个区间：H 
// 0..10 和 156..179（根据光照可调）。
// 范围示例：H:[0,10], S:[43,255], V:[46,255]
#define RED_LOWER1_H 0
#define RED_LOWER1_S 43
#define RED_LOWER1_V 46
#define RED_UPPER1_H 10
#define RED_UPPER1_S 255
#define RED_UPPER1_V 255

#define RED_LOWER2_H 156
#define RED_LOWER2_S 43
#define RED_LOWER2_V 46
#define RED_UPPER2_H 179
#define RED_UPPER2_S 255
#define RED_UPPER2_V 255

// 形态学参数
#define MORPH_KERNEL_SIZE 7
#define MORPH_KERNEL_SIZE1 3

// 灯条识别
#define AREA 60
#define RATIO_MIN 1
#define RATIO_MAX 8
#define ANGLE 40
#define ANGLE_TO_UP 90.0f

// 装甲板识别
#define DISTANCE_MIN 0.2
#define DISTANCE_MAX 5.0
#define ARMOR_ANGLE 40.0f
#define HEIGHT_DIFF 4
#define ANGLE_DIFF 40
#define ARMOR_RATIO_MIN 0.5
#define ARMOR_RATIO_MAX 5

#endif