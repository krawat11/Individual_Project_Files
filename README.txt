IMU Running Gait Analysis
=========================

A pipeline for capturing IMU data over BLE and analysing running gait events
and shank orientation.


Files
-----

arduino_firmware.ino   Arduino sketch for streaming IMU data over BLE.
ble_run_logger.py      BLE host script that logs sensor data to CSV.
preprocessing.py       Low-pass filtering of the IMU channels.
events.py              Detection of foot-strike (IC) and toe-off (TO) events.
orientation.py         Three orientation filters: CF, Madgwick, EKF.
run_demo.py            Loads a CSV and produces the two output figures.


Hardware
--------

Arduino Nano 33 BLE Sense Rev2 with on-board BMI270 IMU.
Sample rate: 100 Hz.


Setup
-----

1. Open arduino_firmware.ino in the Arduino IDE.
2. Install the Arduino libraries: Arduino_BMI270_BMM150 and ArduinoBLE.
3. Select the Nano 33 BLE Sense Rev2 board and upload.

4. For the host scripts, install Python 3.10 or later and:

       pip install bleak numpy scipy matplotlib


Recording data
--------------

   python ble_run_logger.py

The script scans for the device named "RunLogger", connects, and starts
streaming. Press Ctrl+C to stop. A CSV file run_session_<timestamp>.csv
is saved in the current directory.


Running the analysis
--------------------

   python run_demo.py path/to/data.csv

Two figures are written to the figures/ folder:

   event_detection.png    az signal with detected IC and TO markers
   pitch_comparison.png   pitch from CF, Madgwick and EKF filters

Optional flags:

   --start S    start time of the 4-second display window (default 2.0 s)
   --height H   IC peak threshold in g (default 4.0)

For dummy hand-waved sensor data the peaks are usually around 1-2 g, so
add --height 1.0 to make the events trigger.


CSV format
----------

The CSV has 7 columns and a header row:

   timestamp, ax, ay, az, gx, gy, gz

Acceleration is in g, gyroscope in deg/s.
