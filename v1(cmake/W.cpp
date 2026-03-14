#include <iostream>
using namespace std;

int main()
{
    srand((unsigned int)time(NULL));
    int num = rand()% 90 + 10;
    int num_1 = num % 10;
    int num_10 = num / 10;

    int answer;
    cout << "请输入一个两位数：" << endl;
    cin >> answer;
    int answer_1 = answer % 10;
    int answer_10 = answer / 10;

    int money = 0;
    if (answer == num)
    {
        money = 10000;
    }
    else
    {
        if ((answer_10 == num_1) && (answer_1 == num_10 ))
        {
            money = 3000;
        }
        else
        {
            if ((answer_10 == num_10) || (answer_1 == num_1) || 
            (answer_1 == num_10) || (answer_10 == num_1))
            {
                money = 1000;
            }
        }
        
    }
    cout << "随机数字为：" << num << endl;
    cout << "您获得获得的钱：" << money << endl;
    return 0;
}