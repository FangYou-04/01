#include <iostream>
#include <cstring>
using namespace std;


void FZ(char str[], int left, int right)
{
    if (left >= right)
    {
        return;
    }
    else
    {
        char temp = str[left];
        str[left] = str[right];
        str[right] = temp;
        FZ(str, left+1, right-1);
    }
}
int main()
{
    char str[100];
    cout << "请输入字符串:" << endl;
    cin.getline(str, 100);
    int len = strlen(str);
    FZ(str, 0, len-1);
    cout << "反转后字符串为:" << str << endl;
    return 0;
}