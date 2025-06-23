#include "wifidata.h"
#include <fstream>
#include <vector>
#include <sstream>

using namespace std;

wifidata::wifidata() : index(0), lstate("DISCONNECTED") {}

void wifidata::loadCSV(const string &filename)
{
    ifstream file(filename);
    string line;
    if (file && getline(file, line))
    {
        stringstream ss(line);
        string token;
        while (getline(ss, token, ','))
        {
            wifistat.push_back(token);
        }
    }
}
string wifidata::checkstatus()
{
    if (index < wifistat.size())
    {
        string stat = wifistat[index++];
        lstate = stat;
        return stat;
    }
    return lstate;
}
