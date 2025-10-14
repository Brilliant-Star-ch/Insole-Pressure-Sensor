# 📁 Insole-Pressure-Sensor
This project implements a smart insole system using an ESP32 microcontroller, MUX, and multiple pressure sensors.  
The system is designed to collect, log, and visualize real-time pressure data from the insole for gait or balance analysis.  
  
# ⚙️ firmware
1) Insole_Logger_V1:  
    Reads pressure sensor data through the MUX channel.  
    Acquires 16-channel data (per foot) at fixed intervals (e.g., 50 ms).  
    Formats and sends data via serial port for external logging or visualization (Python).  
    Includes frame counters, timestamps, and optional flag bits for synchronization.  
    Can be expanded to save data to SD card  
  
2) Insole_Test:  
     Sequentially scans MUX input channels.  
    Prints sensor readings to Serial Monitor for quick verification.  
    Useful for checking sensor connection, signal range, and analog response.  
    Allows debugging of wiring, power, and MUX switching logic.  
  
3) Usage Flow  
    1️⃣ Run Insole_Test.ino → confirm all sensor channels work.  
    2️⃣ Switch to Insole_Logger_V1.ino → collect and log real-time data.  
    3️⃣ Visualize data via Python script (visualize_data.py).  
  
# 📊 visualization
1) Insole Data:  
   These are data files stored on the SD card through "insole logger v1.ino".  
   Used to visualize at "visualize data.py", it automatically reads the most recent files.  

2) Insole Image  
    Image used to create the outline of the insole.  
3) Insole Outline  
    insole_outline_px.csv: It is contour coordinate data created using "foot image.py"  
    insole_outline_px_filtered.csv: Filter version of the "insole_outline_px.csv" file.  
                                    After filtering the unnecessary coordinates, only the necessary data was left.  
    insole_outline_px_smooth.csv: Based on the coordinate values of "insole_outline_px_filtered.csv", a solid line was created using the archive method.  
                                  The file is created in "foot_image2.py"
4) Foot image  
    Code for creating contours using an insole image.  
5) Foot image2  
    Use the inside outline px filtered.csv file to create a solid outline.  
6) Visualize data  
    Reads the latest log file (COMBO_LOG***.csv) from visualization/insole data/  
    Loads insole outline (insole_outline_px_smooth.csv)  
    Maps sensor coordinates  
    Frame-by-frame visualization (50 ms interval)  
    Dynamic marker size scaling  
    Color-coded heat visualization  
    Excludes invalid frames  
    Draws foot outlines & fills skin tone  
    50 ms pause per frame (plt.pause(0.05))  

# 💼License
This project is licensed under the Creative Commons Attribution-NonCommercial 4.0 International (CC BY-NC 4.0).  
You may use, share, and modify the code for non-commercial purposes with attribution.  
Commercial use is prohibited.  
