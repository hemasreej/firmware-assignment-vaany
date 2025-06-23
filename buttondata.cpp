#include "buttondata.h"
#include <fstream>
#include <sstream>
#include <vector>
#include <iostream>
using namespace std;

buttondata::buttondata() : index(0) {}

void buttondata::loadCSV(const string &filename)
{
    ifstream bfile(filename);
    string bline;
    if (bfile && getline(bfile, bline))
    {
        stringstream bss(bline);
        string btoken;
        while (getline(bss, btoken, ','))
        {
            button.push_back(btoken);
        }
    }
}

string buttondata::checkbutton()
{
    if (index < button.size())
    {
        return button[index++];
    }
    cout << "[Button] Press: NONE" << endl;
    return "NONE";
}
