#include "audiodata.h"
#include <iostream>
using namespace std;

audiodata::audiodata() : wifidata("DISCONNECTED"), ispaused(false) {}

void audiodata::output(const string &filename)
{
    audiofile.open(filename);
    if (!audiofile.is_open())
    {
        cout << "Failed to open " << filename << endl;
    }
    else
    {
        cout << "Opened " << filename << " for writing.\n";
    }
}

void audiodata::wifichange(const string &state)
{
    if (state != wifidata)
    {
        if (state == "CONNECTED")
        {
            write("PLAYING AUDIO ONLINE");
        }
        else
        {
            write("PLAYING AUDIO OFFLINE");
        }
        ispaused = false;
        wifidata = state;
    }
}

void audiodata::buttonpress(const string &input)
{
    if (input == "PAUSE" && !ispaused)
    {
        write("PAUSED");
        ispaused = true;
    }
    else if (input == "PLAY")
    {
        ispaused = false;
    }
    else if (input == "NEXT" || input == "PREVIOUS")
    {
        write("PLAYING THE " + input + " TRACK");
    }
}

void audiodata::playloop()
{
    if (!ispaused)
    {
        write("PLAYING AUDIO");
    }
}

void audiodata::write(const string &message)
{
    std::lock_guard<std::mutex> lock(mt);
    cout << "[AUDIO] Writing: " << message << endl;
    if (audiofile.is_open())
    {
        audiofile << message << std::endl;
    }
}