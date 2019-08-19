#include <iostream>
#include <string>
#include <fstream>

#include "logUtil2.h"

using namespace std;

string getCurrentDateTime(string s)
{
    time_t now = time(0);
    struct tm tstruct;
    char buf[80];
    tstruct = *localtime(&now);
    if(s=="now")
        strftime(buf, sizeof(buf), "%Y-%m-%d %X", &tstruct);
    else if(s=="date")
        strftime(buf, sizeof(buf), "%Y-%m-%d", &tstruct);
    return string(buf);
}

void Logger( string logMsg )
{
    string filePath = "../log_"+getCurrentDateTime("date")+".txt";
    string now = getCurrentDateTime("now");
    ofstream ofs(filePath.c_str(), ios_base::out | ios_base::app );
    ofs << now << '\t' << logMsg << '\n';
    ofs.close();
}
