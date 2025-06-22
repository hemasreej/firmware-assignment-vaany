 Problem Statement

This project simulates the behavior of three firmware modules in an embedded system using C++:

1. **WiFi Module**  
   - Simulates connectivity by reading from `wifi.csv` at 10 Hz  
   - Notifies the Audio Module of connection changes

2. **Button Module**  
   - Simulates button presses (PLAY, PAUSE, NEXT, PREVIOUS) by reading `button.csv` at 0.1 Hz  
   - Sends input events to the Audio Module

3. **Audio Module**  
   - Logs playback state based on inputs from the WiFi and Button modules  
   - Writes all audio activity to `audio.csv`
