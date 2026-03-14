#include <iostream>
using namespace std;

bool ascend(int a,int b)
{
    if (a < b)
    {
        return true;
    }
    
    else
    {
        return false;
    }
}
bool descend(int a,int b)
{
    if (a > b)
    {
        return true;
    }
    else
    {
        return false;
    }   
}

void sortArray(int *arr, int length, bool(*compare)(int, int))
{
    for (int i = 0; i < length; i++)
    {
        for (int j = 0; j < length - 1 -i; j++)
        {
            if(compare(arr[j], arr[j+1]) == false)
            {
                int temp = arr[j];
                arr[j] = arr[j+1];
                arr[j+1] = temp;
            }
        }        
    }
}

int main()
{
    int n;
    cout << "请输入数组长度：" << endl;
    cin >> n;

    int arr[100];
    cout << "请输入" << n << "个整数（用空格隔开）" << endl;
    for (int i = 0; i < n; i++)
    {
        cin >> arr[i];
    }
    
    int sortType;
    cout << "输入排序方式(1.升序,2.降序)" << endl;
    cin >> sortType;

    if (sortType == 1)
    {
        sortArray(arr, n, ascend);
    }
    else
    if(sortType == 2)
    {
        sortArray(arr, n, descend);
    }
    else
    {
        cout << "选择错误，默认升序" <<endl;
        sortArray(arr, n, ascend);
    }
    
    cout << "排序后";
    for (int i = 0; i < n; i++)
    {
        cout << arr[i] << " ";
    }
    return 0;
}