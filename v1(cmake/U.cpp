#include <iostream>
using namespace std;

struct J
{
    char name[20];
    float height;
    float weight;
    float seight;
    float BMI;
};

int main()
{
    J SJ[4]={
        {"尘君饕", 1.80, 70, 5.2, 0},
        {"玛雯凯", 1.80, 66.5, 4.4, 0},
        {"肺耗然", 1.83, 75, 4.8, 0},
        {"礼书茸", 1.90, 74, 5, 0} 
    };
    for (int i = 0; i < 4; i++)
    {
        SJ[i].BMI = SJ[i].weight / (SJ[i].height * SJ[i].weight);
    }
    cout << "学生BMI统计结果:" << endl;
    for (int i = 0; i < 4; i++)
    {
        cout << "姓名: " << SJ[i].name << " BMI: " << SJ[i].BMI << endl;
    }
    return 0;
}
