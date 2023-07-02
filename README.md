# freeIMUCal
IMU Calibration

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
You can connect to your IMU by creating 2 virtual serial port, where the IMU driver sends data to and the calibration program is picking it up

- Linux

See explanation of the different drivers https://www.baeldung.com/linux/make-virtual-serial-port

'''
sudo apt-get install -y socat
socat -d -d pty,rawer,echo=0,link=/tmp/ttyV0 pty,rawer,echo=0,link=/tmp/ttyV1
in one terminal
minicom -D /tmp/ttyV0 -b 115200
in other terminal
minicom -D /tmp/ttyV1 -b 115200
'''

For IMU Calibration use `socat -d -d pty,rawer,echo=0,link=/tmp/ttyV0 pty,rawer,echo=0,link=/tmp/ttyV1&` which creates the ttyV0 for the program reporting IMU data and ttyV1 for freeIMUCal. socat connects the two interfaces via a virtual nullmodem cable meaning sending to ttyV0 will arrive at the ttyV1 receiver and sending to ttyV1 will arrive at tty0 receiver.

- Windows
[com0com](https://sourceforge.net/projects/com0com/)
