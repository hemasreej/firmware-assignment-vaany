#ifndef BUTTON_MODULE_H
#define BUTTON_MODULE_H
#include <string>
#include <vector>

using namespace std;
class buttondata
{
public:
    buttondata();
    void loadCSV(const string &filename);
    string checkbutton();

private:
    vector<string> button;
    int index;
};

#endif