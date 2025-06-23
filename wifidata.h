#ifndef WIFI_MODULE_H
#define WIFI_MODULE_H

#include <vector>
#include <string>
using namespace std;

class wifidata
{
public:
    wifidata();
    void loadCSV(const string &filename);
    string checkstatus();

private:
    vector<string> wifistat;
    int index;
    string lstate;
};
#endif