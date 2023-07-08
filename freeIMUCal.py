"""
freeIMUCal.py - Calibration GUI for IMU devices

This program is based on the FreeIMU Calibration GUI by Fabio Varesano
There were significant changes implemented to make it work with Python 3.7 and PyQt5

Copyright (C) 2023 Urs Utzinger

- Added Gyroscope
- This program just records and displays data. Calibration calculations whould be conducted with ca_lib.py
- Provided option for No 3D plots (as Raspian has incompatible OpenGL version) 
- Updated Serial Data Transfer
- Added ZMQ Data Transfer

Copyright (C) 2012 Fabio Varesano <fabio at varesano dot net>

Development of this code has been supported by the Department of Computer Science,
Universita' degli Studi di Torino, Italy within the Piemonte Project
http://www.piemonte.di.unito.it/

This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

from PyQt5 import uic
from PyQt5.QtCore import QThread, QSettings, pyqtSignal
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtGui import QIcon, QVector3D

import pyqtgraph as pg
import pyqtgraph.opengl as gl

import sys
import numpy as np
import serial, time
import struct
import logging
import pathlib
import zmq
import msgpack

# User Settings
######################################################################
# because on raspian: pyqtgraph.opengl: Requires >= OpenGL 2.0 (not ES); Found b'OpenGL ES 3.1 Mesa 20.3.5'
USE3DPLOT = True
# We dont want to plot every data point
DATADISPLAYINTERVAL = 25 # number of readings to receive before displaying one data point
# SERIAL
BAUDRATE = 115200

acc_range = 15   # +/- Display Range is around 10 m/s^2
mag_range = 100  # +/- Display Range is around60 micro Tesla
gyr_range = 10   # +/- Display Range 33rpm = 33*60rps = 33*60*2pi rad/s = 3.49 rad/s

# Support Function
######################################################################

def hex_to_float(hex_chars):
    '''Unpack 8 bytes to float'''
    hex_bytes = bytes.fromhex(hex_chars)  # Convert hex characters to bytes
    return struct.unpack('!f', hex_bytes)[0]     

######################################################################
# Serial Worker Class
######################################################################

class serialWorker(QThread):
  
  newData              = pyqtSignal(object)
    
  def __init__(self, port, acc_file, gyr_file, mag_file,
                     acc_record, gyr_record, mag_record,
                     acc_append, gyr_append, mag_append, parent=None):

    super().__init__(parent)
    #QThread.__init__(self)
    self.port = port
    self.exiting = False
    self.paused = False
    self.isConnected = False
    self.acc_file_name = acc_file
    self.gyr_file_name = gyr_file
    self.mag_file_name = mag_file
    self.acc_record = acc_record
    self.acc_append = acc_append
    self.gyr_record = gyr_record
    self.gyr_append = gyr_append
    self.mag_record = mag_record
    self.mag_append = mag_append

  def connect(self):
    self.ser = serial.Serial(
      port      = self.port,
      baudrate  = BAUDRATE,
      parity    = serial.PARITY_NONE,
      stopbits  = serial.STOPBITS_ONE,
      bytesize  = serial.EIGHTBITS,
      timeout   = 5.,
      write_timeout = 5.)
    #
    if self.ser.isOpen():
      time.sleep(3)
      msg_out= b'v\r\n'
      j=0
      while self.isConnected == False and j < 0:
        j += 1
        try:
          err = self.ser.write(msg_out) # ask for version
          time.sleep(0.5)
          msg_in = self.ser.read_until(b'\n')
          line_in = msg_in.decode().strip()
          if len(line_in) > 0:
            self.isConnected = True
          else:
            time.sleep(1)
        except: 
          time.sleep(1)
    return self.isConnected
                    
  def run(self):
    
    if self.connect():
      if self.acc_record:
        if self.acc_append: self.acc_file = open(self.acc_file_name, 'a')
        else:               self.acc_file = open(self.acc_file_name, 'w')
      else:                 self.acc_file = None
      if self.gyr_record: 
        if self.gyr_append: self.gyr_file = open(self.gyr_file_name, 'a')
        else:               self.gyr_file = open(self.gyr_file_name, 'w')
      else:                 self.gyr_file = None
      if self.mag_record:
        if self.mag_append: self.mag_file = open(self.mag_file_name, 'a')
        else:               self.mag_file = open(self.mag_file_name, 'w')
      else:                 self.mag_file = None
      
      count = DATADISPLAYINTERVAL # read several values store them and then pass last one to GUI
      in_values = 9 # 3 values for acc, gyr and mag
      readings = [0.0 for i in range(in_values)]

      while not self.exiting:
        if not self.paused:
          self.ser.flushInput()                         # clear serial input buffer
          self.ser.write( ("b{}\r\n".format(count)).encode())    # request data
          for j in range(count):  
            for i in range(in_values):
              byte_array = self.set.read(8)             # byte array of 8 bytes for each floating point value (float is 4 bytes and when converted to readable hex is 8 bytes)
              hex_chars = ''.join(byte_array.decode())     # convert byte array to string
              if len(hex_chars) == 8:                      # 
                readings[i] = hex_to_float(hex_chars)      #
            self.ser.read(2)                            # consumes remaining '\r\n'
            # store all readings in file
            if len(readings) == in_values and not self.paused:
              if self.acc_file != None: 
                acc_readings_line = '{:f} {:f} {:f}\n'.format(readings[0], readings[1], readings[2])
                self.acc_file.write(acc_readings_line)
              if self.gyr_file != None: 
                gyr_readings_line = '{:f} {:f} {:f}\n'.format(readings[3], readings[4], readings[5])
                self.gyr_file.write(gyr_readings_line)
              if self.mag_file != None: 
                mag_readings_line = '{:f} {:f} {:f}\n'.format(readings[6], readings[7], readings[8])
                self.mag_file.write(mag_readings_line)
          # send last reading to display
          if len(readings) == in_values and not self.paused:
            self.newData.emit(readings)
          if self.acc_file != None: self.acc_file.flush()
          if self.gyr_file != None: self.gyr_file.flush()
          if self.mag_file != None: self.mag_file.flush()
        else: # paused
          time.sleep(1)       
       
      # closing acc,gyr and mag files
      if self.acc_file != None: self.acc_file.close()
      if self.gyr_file != None: self.gyr_file.close()
      if self.mag_file != None: self.mag_file.close() 
      self.ser.close()
    else: # did not connect
      pass
              
  def setSamplingStatus(self, status):
    if status == 'Pause':
      self.paused = True
    elif status == 'Continue':
      self.paused = False 
    elif status == 'Stop':
      self.exiting = True
  
  def __del__(self):
    self.exiting = True
    self.wait()

######################################################################
# ZMQ Worker
######################################################################

class zmqWorker(QThread):
  
  newData = pyqtSignal(object)
    
  def __init__(self, port, acc_file, gyr_file, mag_file,
                     acc_record, gyr_record, mag_record,
                     acc_append, gyr_append, mag_append, parent=None):

    super().__init__(parent)
    # QThread.__init__(self)
    
    self.exiting = False
    self.paused = False

    self.zmq_port = port
    self.acc_file_name = acc_file
    self.gyr_file_name = gyr_file
    self.mag_file_name = mag_file
    self.acc_record = acc_record
    self.acc_append = acc_append
    self.gyr_record = gyr_record
    self.gyr_append = gyr_append
    self.mag_record = mag_record
    self.mag_append = mag_append
            
  def run(self):

    self.zmq_context = zmq.Context()
    self.zmq_socket = self.zmq_context.socket(zmq.SUB)
    self.zmq_socket.setsockopt(zmq.SUBSCRIBE, b"imu") # subscribe to all messages
    self.zmq_socket.connect(self.zmq_port)
    
    if self.acc_record:
      if self.acc_append: self.acc_file = open(self.acc_file_name, 'a')
      else:               self.acc_file = open(self.acc_file_name, 'w')
    else:                 self.acc_file = None
    if self.gyr_record: 
      if self.gyr_append: self.gyr_file = open(self.gyr_file_name, 'a')
      else:               self.gyr_file = open(self.gyr_file_name, 'w')
    else:                 self.gyr_file = None
    if self.mag_record:
      if self.mag_append: self.mag_file = open(self.mag_file_name, 'a')
      else:               self.mag_file = open(self.mag_file_name, 'w')
    else:                 self.mag_file = None
    
    count = DATADISPLAYINTERVAL # read several values store them and then pass last one to GUI

    while not self.exiting:
      j = 0
      while j < count:
        response = self.zmq_socket.recv_multipart()
        if len(response) == 2:
          [topic, msg_packed] = response 
          if topic == b'imu':    
            msg_dict = msgpack.unpackb(msg_packed)
            data_imu = dict2obj(msg_dict)
            # check if data is valid
            if not self.paused:
              if hasattr(data_imu, 'acc') and hasattr(data_imu, 'gyr') and hasattr(data_imu, 'mag'): 
                # store readings in files
                if self.acc_file != None: 
                  acc_readings_line = '{:f} {:f} {:f}\n'.format(data_imu.acc.x, data_imu.acc.y, data_imu.acc.z)
                  self.acc_file.write(acc_readings_line)
                if self.gyr_file != None: 
                  gyr_readings_line = '{:f} {:f} {:f}\n'.format(data_imu.gyr.x, data_imu.gyr.y, data_imu.gyr.z)
                  self.gyr_file.write(gyr_readings_line)
                if self.mag_file != None: 
                  mag_readings_line = '{:f} {:f} {:f}\n'.format(data_imu.mag.x, data_imu.mag.x, data_imu.mag.z)
                  self.mag_file.write(mag_readings_line)
                j += 1 # we have a new reading, increment reading counter
      # end while j < count
      # send last reading to display
      if not self.paused:
        self.newData.emit([data_imu.acc.x, data_imu.acc.y, data_imu.acc.z,
                           data_imu.gyr.x, data_imu.gyr.y, data_imu.gyr.z,
                           data_imu.mag.x, data_imu.mag.y, data_imu.mag.z])          

        if self.acc_file != None: self.acc_file.flush()
        if self.gyr_file != None: self.gyr_file.flush()
        if self.mag_file != None: self.mag_file.flush()

    # closing acc,gyr and mag files
    if self.acc_file != None: self.acc_file.close()
    if self.gyr_file != None: self.gyr_file.close()
    if self.mag_file != None: self.mag_file.close() 
    self.zmq_socket.close()
    self.zmq_context.term()

  def setSamplingStatus(self, status):
    if status == 'Pause':
      self.paused = True
    elif status == 'Continue':
      self.paused = False 
    elif status == 'Stop':
      self.exiting = True
  
  def __del__(self):
    self.exiting = True
    self.wait()

class dict2obj:
    def __init__(self, data):
        for key, value in data.items():
            if isinstance(value, dict):
                setattr(self, key, dict2obj(value))
            else:
                setattr(self, key, value)

######################################################################
# Main Program
######################################################################

class FreeIMUCal(QMainWindow):

  dataCollectionStatus = pyqtSignal(str)

  def __init__(self):
    super().__init__()

    self.logger = logging.getLogger('Main')

    # Load UI and setup widgets
    self.ui = uic.loadUi('freeimu_cal.ui', self)
    if USE3DPLOT ==  True:
      self.ui.acc3D.setEnabled(True)
      self.ui.gyr3D.setEnabled(True)
      self.ui.mag3D.setEnabled(True)
    else:
      self.ui.acc3D.setEnabled(False)
      self.ui.gyr3D.setEnabled(False)
      self.ui.mag3D.setEnabled(False)
      
    self.setWindowTitle('FreeIMU Cal')
    current_directory = str(pathlib.Path(__file__).parent.absolute())
    path = current_directory + '/FreeIMU.png'
    self.setWindowIcon(QIcon(path))
    
    # Load user settings
    self.settings = QSettings('FreeIMU Calibration Application', 'Fabio Varesano')

    # Port
    # restore previous port used
    # self.ui.PortEdit.setText(self.settings.value('calgui/PortEdit', '').toString())
    self.ui.PortEdit.setText(self.settings.value('calgui/PortEdit', ''))
    # when user hits enter, we generate the clicked signal to the button so that connection starts
    # self.ui.PortEdit.returnPressed.connect(self.ui.connectButton.click)
    self.ui.PortEdit.returnPressed.connect(lambda: setattr(self, 'port', str(self.ui.PortEdit.text())))

    # Restore Acc,Gyr,Mag File Names
    self.ui.accFile.setText(self.settings.value('calgui/acc_file_name', 'acc.txt'))
    self.ui.gyrFile.setText(self.settings.value('calgui/gyr_file_name', 'gyr.txt'))
    self.ui.magFile.setText(self.settings.value('calgui/mag_file_name', 'mag.txt'))

    self.ui.accFile.setEnabled(True)
    self.ui.gyrFile.setEnabled(True)
    self.ui.magFile.setEnabled(True)

    self.ui.accFile.returnPressed.connect(lambda: setattr(self, 'acc_file_name', self.ui.accFile.text()))
    self.ui.gyrFile.returnPressed.connect(lambda: setattr(self, 'gyr_file_name', self.ui.gyrFile.text()))
    self.ui.magFile.returnPressed.connect(lambda: setattr(self, 'mag_file_name', self.ui.magFile.text()))

    # Buttons
    # Connect
    # self.ui.connectButton.clicked.connect(self.connect)
    # Sampling
    self.ui.samplingToggleButton.clicked.connect(self.sampling_start)
    self.ui.samplingToggleButton.setEnabled(True)
    self.ui.clearButton.clicked.connect(self.clearData)
    self.ui.clearButton.setEnabled(False)
    self.ui.samplingStopButton.setEnabled(False)
    
    self.set_status('Disconnected')

    # Data storages
    ################

    self.acc_data = np.zeros([1,3])
    self.mag_data = np.zeros([1,3])
    self.gyr_data = np.zeros([1,3])

    # Graphs for ACC, GYR and MAG
    ##################################################################

    # Setup graphs 2D

    # ACC
    
    self.ui.accXY.setXRange(-acc_range, acc_range)
    self.ui.accYZ.setXRange(-acc_range, acc_range)
    self.ui.accZX.setXRange(-acc_range, acc_range)
    
    self.ui.accXY.setAspectLocked()
    self.ui.accYZ.setAspectLocked()
    self.ui.accZX.setAspectLocked()

    self.ui.accXY.setBackground('w')
    self.ui.accYZ.setBackground('w')
    self.ui.accZX.setBackground('w')

    self.accXY_sp = pg.ScatterPlotItem([],[], symbol='o', symbolSize=8, pen=pg.mkPen(None), symbolBrush='r')
    self.ui.accXY.addItem(self.accXY_sp)
    self.accYZ_sp = pg.ScatterPlotItem([],[], symbol='o', symbolSize=8, pen=pg.mkPen(None), symbolBrush='g')
    self.ui.accYZ.addItem(self.accYZ_sp)
    self.accZX_sp = pg.ScatterPlotItem([],[], symbol='o', symbolSize=8, pen=pg.mkPen(None), symbolBrush='b')
    self.ui.accZX.addItem(self.accZX_sp)

    # GYR 

    self.ui.gyrXY.setXRange(-gyr_range, gyr_range)
    self.ui.gyrYZ.setXRange(-gyr_range, gyr_range)
    self.ui.gyrZX.setXRange(-gyr_range, gyr_range)
    
    self.ui.gyrXY.setAspectLocked()
    self.ui.gyrYZ.setAspectLocked()
    self.ui.gyrZX.setAspectLocked()

    self.ui.gyrXY.setBackground('w')
    self.ui.gyrYZ.setBackground('w')
    self.ui.gyrZX.setBackground('w')

    self.gyrXY_sp = pg.ScatterPlotItem([],[], symbol='o', symbolSize=8, pen=pg.mkPen(None), symbolBrush='r')
    self.ui.gyrXY.addItem(self.gyrXY_sp)
    self.gyrYZ_sp = pg.ScatterPlotItem([],[], symbol='o', symbolSize=8, pen=pg.mkPen(None), symbolBrush='g')
    self.ui.gyrYZ.addItem(self.gyrYZ_sp)
    self.gyrZX_sp = pg.ScatterPlotItem([],[], symbol='o', symbolSize=8, pen=pg.mkPen(None), symbolBrush='b')
    self.ui.gyrZX.addItem(self.gyrZX_sp)
    
    # MAG

    self.ui.magXY.setXRange(-mag_range, mag_range)
    self.ui.magYZ.setXRange(-mag_range, mag_range)
    self.ui.magZX.setXRange(-mag_range, mag_range)
    
    self.ui.magXY.setAspectLocked()
    self.ui.magYZ.setAspectLocked()
    self.ui.magZX.setAspectLocked()

    self.ui.magXY.setBackground('w')
    self.ui.magYZ.setBackground('w')
    self.ui.magZX.setBackground('w')

    self.magXY_sp = pg.ScatterPlotItem([],[], symbol='o', symbolSize=8, pen=pg.mkPen(None), symbolBrush='r')
    self.ui.magXY.addItem(self.magXY_sp)
    self.magYZ_sp = pg.ScatterPlotItem([],[], symbol='o', symbolSize=8, pen=pg.mkPen(None), symbolBrush='g')
    self.ui.magYZ.addItem(self.magYZ_sp)
    self.magZX_sp = pg.ScatterPlotItem([],[], symbol='o', symbolSize=8, pen=pg.mkPen(None), symbolBrush='b')
    self.ui.magZX.addItem(self.magZX_sp)
    
    # Setup graphs 3D

    # if USE3DPLOT is True:
    if True:

        # ACC 3D
        
        ag = gl.GLGridItem(antialias=True, glOptions='opaque')
        ag.setSize(x=acc_range*2, y=acc_range*2)  # Set the grid size (xSpacing, ySpacing)
        ag.setSpacing(x=acc_range/10, y=acc_range/10)  # Set the grid size (xSpacing, ySpacing)
        ag.setColor((0, 0, 0, 255)) 
        self.ui.acc3D.addItem(ag)

        self.acc3D_sp = gl.GLScatterPlotItem()
        self.acc3D_sp.setGLOptions('translucent')
        self.ui.acc3D.addItem(self.acc3D_sp)

        self.ui.acc3D.opts['center'] = QVector3D(0, 0, 0) 
        self.ui.acc3D.opts['bgcolor'] = (255, 255, 255, 255)  # Set background color to white
        self.ui.acc3D.opts['distance'] = 2*acc_range
        self.ui.acc3D.opts['translucent'] = True
        self.ui.acc3D.show()
        self.ui.acc3D.update()

        # GYR 3D

        gg = gl.GLGridItem(antialias=True, glOptions='opaque')
        gg.setSize(x=gyr_range*2, y=gyr_range*2)  # Set the grid size (xSpacing, ySpacing)
        gg.setSpacing(x=gyr_range/10, y=gyr_range/10)  # Set the grid size (xSpacing, ySpacing)
        gg.setColor((0, 0, 0, 255)) 
        self.ui.gyr3D.addItem(gg)

        self.gyr3D_sp = gl.GLScatterPlotItem()
        self.gyr3D_sp.setGLOptions('translucent')
        self.ui.gyr3D.addItem(self.gyr3D_sp)

        self.ui.gyr3D.opts['center'] = QVector3D(0, 0, 0) 
        self.ui.gyr3D.opts['distance'] = 2*gyr_range
        self.ui.gyr3D.opts['bgcolor'] = (255, 255, 255, 255)  # Set background color to white
        self.ui.gyr3D.opts['translucent'] = True
        self.ui.gyr3D.show()
        self.ui.gyr3D.update()

        # MAG 3D

        mg = gl.GLGridItem(antialias=True, glOptions='opaque')
        mg.setSize(x=mag_range/10, y=mag_range/10)  # Set the grid size (xSpacing, ySpacing)
        mg.setSize(x=mag_range*2, y=mag_range*2)  # Set the grid size (xSpacing, ySpacing)
        mg.setSpacing(x=mag_range/10, y=mag_range/10)  # Set the grid size (xSpacing, ySpacing)
        mg.setColor((0, 0, 0, 255)) 
        self.ui.mag3D.addItem(mg)

        self.mag3D_sp = gl.GLScatterPlotItem()
        self.mag3D_sp.setGLOptions('translucent')
        self.ui.mag3D.addItem(self.mag3D_sp)
 
        self.ui.mag3D.opts['center'] = QVector3D(0, 0, 0) 
        self.ui.mag3D.opts['distance'] = 2*mag_range
        self.ui.mag3D.opts['bgcolor'] = (255, 255, 255, 255)  # Set background color to white
        self.ui.mag3D.opts['translucent'] = True
        self.ui.mag3D.show()
        self.ui.mag3D.update()
      
  def set_status(self, status):
    self.ui.statusbar.showMessage(self.tr(status))
          
  def sampling_start(self):
    
    self.port = str(self.PortEdit.text())
    # save port value to user settings
    self.settings.setValue('calgui/PortEdit', self.port)
    
    self.ui.Protocol.setEnabled(False)

    # Make sure we did not forget to switch to correct protocol
    if 'tcp' in self.port:
      if self.ui.Protocol.currentText() != 'FreeIMU_ZMQ':
        self.ui.Protocol.setCurrentText('FreeIMU_ZMQ')
        
    self.set_status('Starting on ' + self.port)
              
    self.ui.PortEdit.setEnabled(False)
    self.ui.Protocol.setEnabled(False)
    self.ui.samplingToggleButton.setEnabled(True)
    
    self.settings.setValue('calgui/acc_file_name', self.ui.accFile.text())
    self.settings.setValue('calgui/gyr_file_name', self.ui.gyrFile.text())
    self.settings.setValue('calgui/mag_file_name', self.ui.magFile.text())

    self.acc_file_name = self.ui.accFile.text()
    self.gyr_file_name = self.ui.gyrFile.text()
    self.mag_file_name = self.ui.magFile.text()
    
    self.ui.accFile.setEnabled(False)
    self.ui.gyrFile.setEnabled(False)
    self.ui.magFile.setEnabled(False)
    
    self.ui.accRecord.setEnabled(False)
    self.acc_record = self.ui.accRecord.isChecked()
    self.ui.accAppend.setEnabled(False)
    self.acc_append = self.ui.accAppend.isChecked()

    self.ui.gyrRecord.setEnabled(False)
    self.gyr_record = self.ui.gyrRecord.isChecked()
    self.ui.gyrAppend.setEnabled(False)
    self.gyr_append = self.ui.gyrAppend.isChecked()

    self.ui.magRecord.setEnabled(False)
    self.mag_record = self.ui.magRecord.isChecked()
    self.ui.magAppend.setEnabled(False)
    self.mag_append = self.ui.magAppend.isChecked()

    if self.ui.Protocol.currentText() == 'FreeIMU_serial':  
      self.serialWorker = serialWorker(port = self.port, 
                                 acc_file = self.acc_file_name, 
                                 gyr_file = self.gyr_file_name, 
                                 mag_file = self.mag_file_name,
                                 acc_record = self.acc_record,
                                 gyr_record = self.gyr_record,
                                 mag_record = self.mag_record,
                                 acc_append = self.acc_append,
                                 gyr_append = self.gyr_append,
                                 mag_append = self.mag_append)
      self.serialWorker.newData.connect(self.newData)
      self.dataCollectionStatus.connect(self.serialWorker.setSamplingStatus)
      self.serialWorker.start()

    elif self.ui.Protocol.currentText() == 'FreeIMU_ZMQ':
      self.zmqWorker = zmqWorker(   port = self.port, 
                                 acc_file = self.acc_file_name, 
                                 gyr_file = self.gyr_file_name, 
                                 mag_file = self.mag_file_name,
                                 acc_record = self.acc_record,
                                 gyr_record = self.gyr_record,
                                 mag_record = self.mag_record,
                                 acc_append = self.acc_append,
                                 gyr_append = self.gyr_append,
                                 mag_append = self.mag_append)
      self.zmqWorker.newData.connect(self.newData)
      self.dataCollectionStatus.connect(self.zmqWorker.setSamplingStatus)
      self.zmqWorker.start()

    self.ui.set_status('Started Sampling Worker')

    self.ui.samplingToggleButton.setText('Pause Sampling')
    self.ui.samplingToggleButton.clicked.connect(self.sampling_pause)
    self.ui.samplingToggleButton.setEnabled(True)

    self.ui.samplingStopButton.setEnabled(True)
    self.ui.samplingStopButton.clicked.connect(self.sampling_end)
    
    self.ui.clearButton.setEnabled(True)

  def sampling_pause(self):
    self.dataCollectionStatus.emit('Pause')
    self.ui.set_status('Paused Sampling Worker')
    self.ui.samplingToggleButton.setText('Resume Sampling')
    self.ui.samplingToggleButton.clicked.connect(self.sampling_resume)
    self.ui.samplingToggleButton.setEnabled(True)

  def sampling_resume(self):
    self.dataCollectionStatus.emit('Continue')
    self.ui.set_status('Continued Sampling Worker')
    self.ui.samplingToggleButton.setText('Pause Sampling')
    self.ui.samplingToggleButton.clicked.connect(self.sampling_pause)
    self.ui.samplingToggleButton.setEnabled(True)
    
  def sampling_end(self):

    self.dataCollectionStatus.emit('Stop')

    self.ui.set_status('Stopped Sampling Worker')
    self.ui.samplingToggleButton.setText('Start Sampling')
    self.ui.samplingToggleButton.clicked.connect(self.sampling_start)
    self.ui.samplingToggleButton.setEnabled(True)
    
    self.ui.samplingStopButton.clicked.connect(self.sampling_end)
    self.ui.samplingStopButton.setEnabled(False)

    self.ui.Protocol.setEnabled(True)
    self.ui.PortEdit.setEnabled(True)
    self.ui.Protocol.setEnabled(True)
    
    self.ui.samplingToggleButton.setEnabled(True)
    self.ui.samplingStopButton.setEnabled(False)

    self.ui.accFile.setEnabled(True)
    self.ui.gyrFile.setEnabled(True)
    self.ui.magFile.setEnabled(True)
    self.ui.accRecord.setEnabled(True)
    self.ui.accAppend.setEnabled(True)
    self.ui.gyrRecord.setEnabled(True)
    self.ui.gyrAppend.setEnabled(True)
    self.ui.magRecord.setEnabled(True)
    self.ui.magAppend.setEnabled(True)
    
    
  def newData(self, reading):
    # display data
    if self.ui.accDisplay.isChecked():
      self.acc_data = np.concatenate((self.acc_data, np.array([[reading[0],reading[1],reading[2]]])), axis=0)
    if self.ui.gyrDisplay.isChecked():
      self.gyr_data = np.concatenate((self.gyr_data, np.array([[reading[3],reading[4],reading[5]]])), axis=0)
    
    if self.ui.magDisplay.isChecked():
      self.mag_data = np.concatenate((self.mag_data, np.array([[reading[6],reading[7],reading[8]]])), axis=0)
    
    self.accXY_sp.setData(x=self.acc_data[1:-1,0], y=self.acc_data[1:-1,1])
    self.accYZ_sp.setData(x=self.acc_data[1:-1,1], y=self.acc_data[1:-1,2])
    self.accZX_sp.setData(x=self.acc_data[1:-1,2], y=self.acc_data[1:-1,0])

    self.gyrXY_sp.setData(x=self.gyr_data[1:-1,0], y=self.gyr_data[1:-1,1])
    self.gyrYZ_sp.setData(x=self.gyr_data[1:-1,1], y=self.gyr_data[1:-1,2])
    self.gyrZX_sp.setData(x=self.gyr_data[1:-1,2], y=self.gyr_data[1:-1,0])

    self.magXY_sp.setData(x=self.mag_data[1:-1,0], y=self.mag_data[1:-1,1])
    self.magYZ_sp.setData(x=self.mag_data[1:-1,1], y=self.mag_data[1:-1,2])
    self.magZX_sp.setData(x=self.mag_data[1:-1,2], y=self.mag_data[1:-1,0])

    if USE3DPLOT == True:
        color = (1., 0., 0., 0.5)
        size = 10.
        self.acc3D_sp.setData(pos=self.acc_data[1:-1,:], color = color, size=size)
        self.gyr3D_sp.setData(pos=self.gyr_data[1:-1,:], color = color, size=size)
        self.mag3D_sp.setData(pos=self.mag_data[1:-1,:], color = color, size=size)

        # Calculate the data range
        if len(self.acc_data) > 2:
            data_min = np.min(self.acc_data)
            data_max = np.max(self.acc_data)
            data_range = np.linalg.norm(data_max - data_min)
            # Set up the view and zoom to fit the data
            camera_distance = data_range * 2
            self.ui.acc3D.opts['distance'] = camera_distance
        self.ui.acc3D.update()
        
        if len(self.gyr_data) > 2:
            data_min = np.min(self.gyr_data)
            data_max = np.max(self.gyr_data)
            data_range = np.linalg.norm(data_max - data_min)
            # Set up the view and zoom to fit the data
            camera_distance = data_range * 2
            self.ui.gyr3D.opts['distance'] = camera_distance
        self.ui.gyr3D.update()
        
        if len(self.mag_data) > 2:
            data_min = np.min(self.mag_data)
            data_max = np.max(self.mag_data)
            data_range = np.linalg.norm(data_max - data_min)
            # Set up the view and zoom to fit the data
            camera_distance = data_range * 2
            self.ui.mag3D.opts['distance'] = camera_distance
        self.ui.mag3D.update()

  def clearData(self):
    # display data
    if self.ui.accDisplay.isChecked():
      self.acc_data = np.zeros([1,3])
      
    if self.ui.gyrDisplay.isChecked():
      self.gyr_data = np.zeros([1,3])
      
    if self.ui.magDisplay.isChecked():
     self.mag_data = np.zeros([1,3])
     
    self.accXY_sp.setData(x=self.acc_data[1:-1,0], y=self.acc_data[1:-1,1])
    self.accYZ_sp.setData(x=self.acc_data[1:-1,1], y=self.acc_data[1:-1,2])
    self.accZX_sp.setData(x=self.acc_data[1:-1,2], y=self.acc_data[1:-1,0])

    self.gyrXY_sp.setData(x=self.gyr_data[1:-1,0], y=self.gyr_data[1:-1,1])
    self.gyrYZ_sp.setData(x=self.gyr_data[1:-1,1], y=self.gyr_data[1:-1,2])
    self.gyrZX_sp.setData(x=self.gyr_data[1:-1,2], y=self.gyr_data[1:-1,0])

    self.magXY_sp.setData(x=self.mag_data[1:-1,0], y=self.mag_data[1:-1,1])
    self.magYZ_sp.setData(x=self.mag_data[1:-1,1], y=self.mag_data[1:-1,2])
    self.magZX_sp.setData(x=self.mag_data[1:-1,2], y=self.mag_data[1:-1,0])

    if USE3DPLOT == True:
        self.acc3D_sp.setData(pos=self.acc_data, color = (1, 1, 1, 1), size=2)
        self.gyr3D_sp.setData(pos=self.gyr_data, color = (1, 1, 1, 1), size=2)
        self.mag3D_sp.setData(pos=self.mag_data, color = (1, 1, 1, 1), size=2)
    
######################################################################
# Main Program
######################################################################

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = FreeIMUCal()
    window.show()
    sys.exit(app.exec_())
