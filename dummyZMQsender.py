#!/usr/bin/python3

################################################################
# Dummy ZMQ Sender
################################################################
            
# IMPORTS
################################################################
import asyncio
import logging
import argparse
import signal
import random
import time
import os
import asyncio
import zmq
# import zmq.asyncio
import msgpack

from pyIMU.quaternion import Vector3D
from copy import copy

timerResolution = time.get_clock_info('monotonic').resolution

DATAINTERVAL   = 1./100. - timerResolution/8.
ZMQPORT        = 5556
REPORTINTERVAL = 1./10.  - timerResolution/4.

################################################################
# Support Functions
################################################################

class imuData(object):
    '''Raw Data from the sensor'''
    def __init__(self, 
                 time: float=0.0,
                 acc: Vector3D = Vector3D(0.,0.,0.),
                 gyr: Vector3D = Vector3D(0.,0.,0.),
                 mag: Vector3D = Vector3D(0.,0.,0.)
                ) -> None:        
        self.time = time
        self.acc  = acc
        self.mag  = mag
        self.gyr  = gyr

def obj2dict(obj):
    # decoding object variables to nested dict 
    if isinstance(obj, dict):
        return {k: obj2dict(v) for k, v in obj.items()}
    elif hasattr(obj, '__dict__'):
        return obj2dict(vars(obj))
    elif isinstance(obj, list):
        return [obj2dict(item) for item in obj]
    else:
        return obj

################################################################
# Dummy
################################################################

class dummyZMQ:
            
    def __init__(self, logger=None) -> None:

        # Signals
        self.dataAvailable          = False
        self.finish_up              = False
        
        if logger is not None: self.logger = logger
        else:                  self.logger = logging.getLogger('gearVRC')

        # device sensors
        self.sensorTime             = 0.    # There are 3 different times transmitted from the sensor
        self.previous_sensorTime    = 0.

        # IMU

        # Timing
        ###################
        self.data_deltaTime         = 0.
        self.data_rate              = 0
        self.data_updateInterval    = DATAINTERVAL

        self.zmq_deltaTime          = 0.
        self.zmq_rate               = 0

        self.report_deltaTime       = 0.
        self.report_rate            = 0
        self.report_updateInterval  = REPORTINTERVAL 
        
        self.acc           = Vector3D(0.,0.,0.)
        self.gyr           = Vector3D(0.,0.,0.)
        self.mag           = Vector3D(0.,0.,0.)

    async def update(self):
        '''
        Generate normalized radom IMU data
        '''

        self.logger.log(logging.INFO, 'Starting ...')

        data_lastTimeRate   = time.perf_counter()
        previous_dataUpdate = time.perf_counter()
        data_updateCounts   = 0

        report_lastTimeRate = time.perf_counter()
        previous_reportUpdate = time.perf_counter()
        report_updateCounts = 0

        zmq_lastTimeRate   = time.perf_counter()
        previous_zmqUpdate = time.perf_counter()
        zmq_updateCounts   = 0
        
        context = zmq.Context()      
        socket = context.socket(zmq.PUB)
        socket.bind("tcp://*:{}".format(args.zmqport))
        
        data_IMU     = imuData()

        while not self.finish_up:

            currentTime = time.perf_counter()
            
            self.data_deltaTime = currentTime - previous_dataUpdate
            previous_dataUpdate = copy(currentTime)

            data_updateCounts += 1
            if (currentTime - data_lastTimeRate)>= 1.:
                self.data_rate = copy(data_updateCounts)
                data_lastTimeRate = copy(currentTime)
                data_updateCounts = 0

            accX = random.random()*2.-1.
            accY = random.random()*2.-1.
            accZ = random.random()*2.-1.
            gyrX = random.random()*2.-1.
            gyrY = random.random()*2.-1.
            gyrZ = random.random()*2.-1.
            magX = random.random()*2.-1.
            magY = random.random()*2.-1.
            magZ = random.random()*2.-1.
            
            self.sensorTime = copy(currentTime)
            self.acc = Vector3D(accX, accY, accZ)
            self.gyr = Vector3D(gyrX, gyrY, gyrZ)
            self.mag = Vector3D(magX, magY, magZ)
            
            self.acc.normalize()
            self.gyr.normalize()
            self.mag.normalize()
            
            self.acc *=9.81
            self.gyr *=5.
            self.mag *=40.

            ##############################################################
                
            self.zmq_deltaTime = currentTime - previous_zmqUpdate
            previous_zmqUpdate = copy(currentTime)

            zmq_updateCounts += 1
            if (currentTime - zmq_lastTimeRate)>= 1.:
                self.zmq_rate = copy(zmq_updateCounts)
                zmq_lastTimeRate = copy(currentTime)
                zmq_updateCounts = 0

            # format the IMU data
            data_IMU.time = self.sensorTime
            data_IMU.acc  = self.acc
            data_IMU.gyr  = self.gyr
            data_IMU.mag  = self.mag
            imu_msgpack = msgpack.packb(obj2dict(vars(data_IMU)))
            socket.send_multipart([b"imu", imu_msgpack])

            ##############################################################

            if args.report > 0:

                if (currentTime - previous_reportUpdate) >= self.report_updateInterval:
                    self.report_deltaTime = currentTime - previous_reportUpdate
                    previous_reportUpdate = copy(currentTime)

                    report_updateCounts += 1
                    if (currentTime - report_lastTimeRate)>= 1.:
                        self.report_rate = copy(report_updateCounts)
                        report_lastTimeRate = copy(currentTime)
                        report_updateCounts = 0

                    # Display the Data
                    msg_out = '-------------------------------------------------\n'
                    msg_out+= 'dummy ZMW IMU Generator\n'
                    msg_out+= '-------------------------------------------------\n'

                    msg_out+= 'Data    {:>10.6f}, {:>3d}/s\n'.format(self.data_deltaTime*1000.,        self.data_rate)
                    msg_out+= 'Report  {:>10.6f}, {:>3d}/s\n'.format(self.report_deltaTime*1000.,      self.report_rate)
                    if args.zmqport is not None:
                        msg_out+= 'ZMQ     {:>10.6f}, {:>3d}/s\n'.format(self.zmq_deltaTime*1000.,     self.zmq_rate)
                    msg_out+= '-------------------------------------------------\n'

                    msg_out+= 'Time: {:>10.6f}, dt: {:>10.6f}\n'.format(self.sensorTime, self.data_deltaTime)
                    msg_out+= 'Accel {:>8.3f} {:>8.3f} {:>8.3f}\n'.format(self.acc.x,self.acc.y,self.acc.z)
                    msg_out+= 'Gyro  {:>8.3f} {:>8.3f} {:>8.3f}\n'.format(self.gyr.x,self.gyr.y,self.gyr.z)
                    msg_out+= 'Magno {:>8.3f} {:>8.3f} {:>8.3f}\n'.format(self.mag.x,self.mag.y,self.mag.z)

                    print(msg_out, flush=True)

            ##############################################################
                        
            # Wait to next interval time
            sleepTime = self.data_updateInterval - (time.perf_counter() - currentTime)
            await asyncio.sleep(max(0.,sleepTime))
            timingError = time.perf_counter() - currentTime - self.data_updateInterval
            self.data_updateInterval = max(0., DATAINTERVAL - timingError)
            
        self.logger.log(logging.INFO, 'Stopping')
                
    async def handle_termination(self, tasks:None):
        '''
        Stop the task loops
        Stop the sensor
        Cancel tasks if list is provided which will speed up closing of program
        '''
        self.finish_up = True
        if tasks is not None: # This will terminate tasks faster
            self.logger.log(logging.INFO, 'Cancelling all Tasks...')
            await asyncio.sleep(0.5) # give some time for tasks to finish up
            for task in tasks:
                if task is not None:
                    task.cancel()

async def main(args: argparse.Namespace):

    # Setup logging
    logger = logging.getLogger(__name__)
    logger.log(logging.INFO, 'Starting dummy ZMQ IMU Generator')
    # gearVRC Controller
    controller = dummyZMQ(logger=logger)

    main_task     = asyncio.create_task(controller.update()) 
             
    # Set up a Control-C handler to gracefully stop the program
    # This mechanism is only available in Unix
    if os.name == 'posix':
        # Get the main event loop
        loop = asyncio.get_running_loop()
        loop.add_signal_handler(signal.SIGINT,  lambda: asyncio.create_task(controller.handle_termination(tasks=[main_task])) ) # control-c
        loop.add_signal_handler(signal.SIGTERM, lambda: asyncio.create_task(controller.handle_termination(tasks=[main_task])) ) # kill

    # Wait until all tasks are completed, which is when user wants to terminate the program
    await asyncio.wait([main_task], timeout=float('inf'))

    logger.log(logging.INFO,'Exit')

if __name__ == '__main__':
    
    parser = argparse.ArgumentParser()

    device_group = parser.add_mutually_exclusive_group(required=False)

    parser.add_argument(
        '-r',
        '--report',
        dest = 'report',
        type = int,
        metavar='<report>',
        help='report level: 0(None), 1(regular)',
        default = 0
    )

    parser.add_argument(
        '-z',
        '--zmq',
        dest = 'zmqport',
        type = int,
        metavar='<zmqport>',
        help='port used by ZMQ, e.g. 5556',
        default = 5556
    )

    parser.add_argument(
        '-d',
        '--debug',
        action='store_true',
        help='sets the log level from info to debug',
        default = False
    )

    args = parser.parse_args()
        
    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(
        level=log_level,
        format='%(asctime)-15s %(levelname)s: %(message)s'
    )   
    
    try:
        asyncio.run(main(args))
    except KeyboardInterrupt:
        pass
