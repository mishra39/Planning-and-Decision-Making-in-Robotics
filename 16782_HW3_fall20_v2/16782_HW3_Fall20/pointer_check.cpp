#include<iostream>
#include<vector>

using namespace std;

int main()
{
    int *ptr;
    int *num1 = new int();
    vector<int*> vect;

    *num1 = 10;
    ptr   = num1;
    *ptr = 5;
    vect.push_back(ptr);
    cout << "Num1: " << *num1  << " ptr: " << *ptr << endl;
    int num2 = 55;
    *ptr = 10;// &num2;
    cout << "Ptr now has value: " << *ptr << endl;
    for (auto i:vect)
    {
        cout << "Vector contains: " << *i;
    }
    cout << endl;

    int num3 = 9;
    ptr = &num3;
    cout << "Ptr now has value: " << *ptr << endl;
    num3 = 20;
    cout << "Ptr now has value: " << *ptr << endl;
    delete num1;
}