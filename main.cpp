#include <iostream>
#include <thread>
#include <chrono>

#include "wifidata.h"
#include "buttondata.h"
#include "audiodata.h"

using namespace std;

int main()
{

    wifidata wifi;
    buttondata button;
    audiodata audio;

    // Load CSV inputs
    wifi.loadCSV("wifi.csv");
    button.loadCSV("button.csv");
    audio.output("AAudio.csv");
    cout << " output() called" << endl;

    int tick = 0;

    while (true)
    {
        string wifistate = wifi.checkstatus();
        audio.wifichange(wifistate);

        if (tick % 100 == 0)
        {
            string btn = button.checkbutton();
            if (btn != "NONE")
            {
                audio.buttonpress(btn);
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
