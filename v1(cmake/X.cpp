#include <iostream>
using namespace std;

class ArraySatats
{
private:
    int *arr;
    int length;
public:
    ArraySatats(const int *inputArr, int inputlength)
    {
        length = inputlength;
        if (length > 0)
        {
            arr = new int[length];
            for (int i = 0; i < length; i++)
            {
                arr[i] = inputArr[i];
            }
            
        }
        else
        {
            arr = NULL;
        }    
    };
    ~ArraySatats()
    {
        delete[] arr;
    };

    int getMax()
    {
        if (length == 0)
        {
            cout << "数组为空,最大值返回0" << endl;
            return 0;
        }
        int Max = arr[0];
        for (int i = 0; i < length; i++)
        {
            if (arr[i] > Max)
            {
                Max = arr[i];
            }
        }
        return Max;
    }

    int getMin()
    {
        if (length == 0)
        {
            cout << "数组为空,最小值返回0" << endl;
            return 0;
        }
        int Min = arr[0];
        for (int i = 0; i < length; i++)
        {
            if (arr[i] == Min)
            {
                Min = arr[i];
            }
        }
        return Min;
    }

    double getAverage()
    {
        if (length == 0)
        {
            cout << "数组为空,平均值返回0.0" << endl;
            return 0;
        }
        int All = 0;
        for (int i = 0; i < length; i++)
        {
            All += arr[i];
        }
        return (double)All / length;
    }
};
