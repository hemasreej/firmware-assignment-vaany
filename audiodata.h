#ifndef AUDIO_MODULE_H
#define AUDIO_MODULE_H
#include <string>
#include <mutex>
#include <fstream>

using namespace std;

class audiodata
{
public:
    audiodata();
    void output(const string &filename);
    void wifichange(const string &state);
    void buttonpress(const string &input);
    void playloop();

private:
    mutex mt;
    bool ispaused;
    ofstream audiofile;
    string wifidata;

    void write(const string &message);
};

#endif