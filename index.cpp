#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <thread>
#include <chrono>
#include <mutex>

using namespace std;

class wifidata
{
public:
    vector<string> wifistate;
    int index = 0;
    string lstate = "DISCONNECTED";

    wifidata(const string &filename)
    {
        readCSV(filename, wifistate);
    }

    string checkstatus()
    {
        if (index < wifistate.size())
        {
            string stat = wifistate[index++];
            lstate = stat;
            return stat;
        }
        return lstate;
    }

private:
    void readCSV(const string &filename, vector<string> &data)
    {
        ifstream file(filename);
        string line;
        if (file && getline(file, line))
        {
            stringstream ss(line);
            string token;
            while (getline(ss, token, ','))
            {
                data.push_back(token);
            }
        }
    }
};

class buttondata
{
public:
    vector<string> buttonstate;
    int index = 0;

    buttondata(const string &filename)
    {
        readCSV(filename, buttonstate);
    }

    string checkbutton()
    {
        if (index < buttonstate.size())
        {
            return buttonstate[index++];
        }
        return "NONE";
    }

private:
    void readCSV(const string &filename, vector<string> &bout)
    {
        ifstream bfile(filename);
        string bline;
        if (bfile && getline(bfile, bline))
        {
            stringstream bss(bline);
            string btoken;
            while (getline(bss, btoken, ','))
            {
                bout.push_back(btoken);
            }
        }
    }
};

class audiodata
{
public:
    string wstate = "DISCONNECTED";
    bool ispaused = false;
    ofstream audiofile;
    mutex mt;

    audiodata(const string &filename)
    {
        audiofile.open(filename);
    }

    ~audiodata()
    {
        if (audiofile.is_open())
        {
            audiofile.close();
        }
    }

    void onwifichangestat(const string &nstate)
    {
        if (nstate != wstate)
        {
            if (nstate == "CONNECTED")
            {
                write("PLAYING AUDIO ONLINE");
                ispaused = false;
            }
            else if (nstate == "DISCONNECTED")
            {
                write("PLAYING AUDIO OFFLINE");
                ispaused = false;
            }
            wstate = nstate;
        }
    }

    void onbuttonpress(const string &button)
    {
        if (button == "PAUSE")
        {
            if (!ispaused)
            {
                write("PAUSED");
                ispaused = true;
            }
        }
        else if (button == "PLAY")
        {
            ispaused = false;
        }
        else if (button == "NEXT" || button == "PREVIOUS")
        {
            string track = (button == "NEXT") ? "NEXT" : "PREVIOUS";
            write("PLAYING THE " + track + " TRACK");
        }
    }

    void playloop()
    {
        if (!ispaused)
        {
            write("PLAYING AUDIO");
        }
    }

private:
    void write(const string &message)
    {
        lock_guard<mutex> lock(mt);
        if (audiofile.is_open())
        {
            audiofile << message << endl;
        }
    }
};

int main()
{
    wifidata wifi("wifi.csv");
    buttondata button("button.csv");
    audiodata audio("audio.csv");

    int tick = 0;

    while (true)
    {
        string wifist = wifi.checkstatus();
        audio.onwifichangestat(wifist);

        if (tick % 100 == 0)
        {
            string btn = button.checkbutton();
            if (btn != "NONE")
            {
                audio.onbuttonpress(btn);
            }
        }

        audio.playloop();
        this_thread::sleep_for(chrono::milliseconds(100));
        tick++;

        if (tick > 1000)
            break;
    }

    return 0;
}
