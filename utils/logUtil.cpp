#include <iostream>
#include <cstdio>
#include <string>

#include "logUtil.h"

using namespace std;

void log(string msg)
{
    freopen("output1.txt", "w", stdout);
    cout << msg << endl;
}



