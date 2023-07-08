# freeIMUCal
IMU Calibration using ZMQ or serial connection to an IMU. The resulting offsets and axis scales will improve the IMU accuracy.

## Background
This work is based on Fabio Varesano's [implementation](https://www.researchgate.net/publication/258817923_FreeIMU_An_Open_Hardware_Framework_for_Orientation_and_Motion_Sensing) of [FreeIMU](https://github.com/Fabio-Varesano-Association/freeimu)

## Dependencies
- PyOpenGL `sudo apt-get install python3-PyOpenGL` or `pip install PyOpenGl`
- PyQt5 `sudo apt-get install python3-PyQt5`  or `pip install PyQt5`
- pyqtgraph `sudo apt-get install python3-pyqtgraph`  or `pip install pyqtgraph`

Files needed from this distribution:
- cal_lib.py which includes ellipsoid_fit to fit ellipsoid to [x,y,z] data
- freeimu_cal.ui user interface with 3D graphs
- freeimu_cal_noGL.ui userinterface for machines were pyqtgraph can not create 3D scatter plot

## Installing Virtual Serial Port
You can connect to your IMU by creating 2 virtual serial port, where the IMU driver sends data to and the calibration program is picking it up from

- Linux

See explanation of the different virtual serial drivers https://www.baeldung.com/linux/make-virtual-serial-port

```
sudo apt-get install -y socat
socat -d -d pty,rawer,echo=0,link=/tmp/ttyV0 pty,rawer,echo=0,link=/tmp/ttyV1
in one terminal
minicom -D /tmp/ttyV0 -b 115200
in other terminal
minicom -D /tmp/ttyV1 -b 115200
```

For IMU Calibration use `socat -d -d pty,rawer,echo=0,link=/tmp/ttyV0 pty,rawer,echo=0,link=/tmp/ttyV1&` which creates ttyV0 for the program reporting IMU data and ttyV1 for freeIMUCal to receive data. socat connects the two interfaces via a virtual nullmodem cable meaning sending to ttyV0 will arrive at the ttyV1 receiver and sending to ttyV1 will arrive at tty0 receiver.

- Windows
[com0com](https://sourceforge.net/projects/com0com/)

## Usage
I use this program to record data.
I use the main section in ```cal_lib.py``` to create the correction JSON files.

There are up to three sensors to calibrate:
- `Magnetometer`

The magnetometer needs to be calibrated in a magnet and motor free environment. You should obtain a good distribution of numbers on all locations along the sphere. Usually there is significant offset.

- `Gyroscope`

For the gyroscope you need a turn table used to play vinyl records. It has a setting for 33.333 RPM and 45 RPM usually the players are accurate. You will need to place the sensor on the table in different orientations and record the data in steps by pausing recording when you change the orientation. You should give the turntable some time to adjust the speed. To record the offset you can include recording when the turn table is off and sort that data in the analysis file. Its difficult to obtain a large number of orientations and its best to just fit the scales and not cross axis sensitivity.

- `Accelerometer`

You will need to gently waggle the sensor. You will want to cover as many sections of the spherical surface as possible by systematically rotate the sensor. You can record the accelerometer and magnetometer simultaneously. You can not use gyroscope readings during these readings.

The calibration fits should produce an offset as well as scaling in all three directions. You can enable fit option to compute cross axis sensitivity. Its unlikely that the cross axis sensitivity is significant compared to orientation shifts between the three sensors. The scales are the diagonal elements of the correction matrix.
