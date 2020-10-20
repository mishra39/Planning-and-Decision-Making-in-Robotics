#include <random>
#include <vector>
#include <iostream>

using namespace std;
#define PI 3.14

int main()
{
  vector<double> config;
  random_device rd;
  default_random_engine gen(rd());
  uniform_real_distribution<double> distribution(-PI, PI);
  cout << config.size()<< endl;
  for (int i = 0 ; i < 5; i++)
  {
    double pose = distribution(gen);
    config.push_back(pose);
    //cout << pose << endl;
    cout << config.size()<< endl;
  }
}