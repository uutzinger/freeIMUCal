"""
cal_lib.py - Ellipsoid into Sphere calibration library based upon numpy and linalg
Copyright (C) 2012 Fabio Varesano <fabio at varesano dot net>

Updates by Urs Utzinger to include ellipsoid fit from Yury Petrov <yurypetrov at gmail dot com>

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

import numpy as np
import json

def calibrate(data, option):
    
    (center, radii, evecs, v)= ellipsoid_fit(data, option=option) # assume no cross sensitivity
 
    min_r = min(radii)
    scaleMat = np.array([[min_r/radii[0], 0, 0], [0, min_r/radii[1], 0], [0, 0,  min_r/radii[2]]])
    
    correctionMat = evecs * scaleMat * evecs.T
            
    return (center, correctionMat, radii)

    # H = np.array([x, y, z, -y**2, -z**2, np.ones([len(x), 1])])
    # H = np.transpose(H)
    # w = x**2

    # # solving H * a = w for a
    # # a0*x + a1*y + a2*z - a3*y^2 - a4*z^2 -x^2 = -a5
    # # Ax^2 + By^2 + Cz^2 + 2Gx + 2Hy + 2Iz = 1

    # (X, residues, rank, shape) = linalg.lstsq(H, w)

    # OSx = X[0] / 2
    # OSy = X[1] / (2 * X[3])
    # OSz = X[2] / (2 * X[4])

    # A = X[5] + OSx**2 + X[3] * OSy**2 + X[4] * OSz**2
    # B = A / X[3]
    # C = A / X[4]

    # SCx = np.sqrt(A)
    # SCy = np.sqrt(B)
    # SCz = np.sqrt(C)

    # # type conversion from numpy.float64 to standard python floats
    # offsets = [OSx, OSy, OSz]
    # scale   = [SCx, SCy, SCz]

    # offsets = map(np.asscalar, offsets)
    # scale   = map(np.asscalar, scale)

    # return (offsets, scale)

def calibrate_from_file(file_name):
    '''read data from file, each line has 3 floats'''
    data=readfromFile(file_name)
    (center, correctionMat, radii) = calibrate(data, option=1)
    return (center, correctionMat)

def readfromFile(file_name):
    '''read data from file, each line has 3 floats'''
    samples_f = open(file_name, 'r')
    samples_x = []
    samples_y = []
    samples_z = []
    for line in samples_f:
        reading = line.split()
        if len(reading) == 3:
            samples_x.append(float(reading[0]))
            samples_y.append(float(reading[1]))
            samples_z.append(float(reading[2]))
        
    return np.array([samples_x, samples_y, samples_z]).T

def cleanupData(data, norm_min, norm_max):
    # Remove Data outside the acceptable range
    data_norm = np.linalg.norm(data,axis=1) 
    accept = np.logical_and((data_norm>=norm_min), (data_norm<norm_max))
    return data[accept,:]

def removeOutlayers(data):
    # Accept data in the range of mean+/-3*std
    data_norm = np.linalg.norm(data,axis=1)
    data_average=np.average(data_norm)
    data_std=np.std(data_norm)    
    accept = np.logical_and((data_norm>=data_average-3.*data_std), (data_norm<data_average+3*data_std))
    return data[accept,:]

def saveCalibration(filename, center, correctionMat):
    #
    d = {
    "offset_x": center[0], 
    "offset_y": center[1], 
    "offset_z": center[2], 
    "cMat_00":  correctionMat[0,0],
    "cMat_01":  correctionMat[0,1],
    "cMat_02":  correctionMat[0,2],
    "cMat_10":  correctionMat[1,0],
    "cMat_11":  correctionMat[1,1],
    "cMat_12":  correctionMat[1,2],
    "cMat_20":  correctionMat[2,0],
    "cMat_21":  correctionMat[2,1],
    "cMat_22":  correctionMat[2,2]
    }

    with open(filename, 'w') as file:
        json.dump(d, file)

def readCalibration(filename, center, correctionMat):
    #
    with open(filename, 'r') as file:
        d = json.load(file)

    center = np.array([d['offset_x'],d['offset_y'],d['offset_z']])
    correctionMat = np.empty([3,3])

    correctionMat[0,0] = d['cMat_00']
    correctionMat[0,1] = d['cMat_01']
    correctionMat[0,2] = d['cMat_02']
    correctionMat[1,0] = d['cMat_10']
    correctionMat[1,1] = d['cMat_11']
    correctionMat[1,2] = d['cMat_12']
    correctionMat[2,0] = d['cMat_20']
    correctionMat[2,1] = d['cMat_21']
    correctionMat[2,2] = d['cMat_22']

    return center, correctionMat
    
def computeCalibratedData(data, offsets, correctionMat):
    #
    output = None

    if isinstance(data, np.ndarray):
        if data.shape[-1] == 3:
            if isinstance(offsets, np.ndarray):
                o_shape = offsets.shape
                if o_shape[-1] == 3 and len(o_shape)==1:
                    if data is not None:
                        output = data - offsets # subtract offsets

    if isinstance(correctionMat, np.ndarray):
        c_shape = correctionMat.shape
        if len(c_shape) == 2:
            if c_shape[0] == 3 and c_shape[1] == 3:
                if output is not None:
                    output = output @ correctionMat # apply scale and cross axis sensitivity

    return output

def ellipsoid(center,radii):
    u = np.linspace(0,2*np.pi, 100)
    v = np.linspace(0,np.pi, 100)
    x = center[0] + radii[0]*np.outer(np.cos(u), np.sin(v))
    y = center[1] + radii[1]*np.outer(np.sin(u), np.sin(v))
    z = center[2] + radii[2]*np.outer(np.ones_like(u), np.cos(v))
    return(x,y,z)
        
def ellipsoid_fit(data, **kwargs):
    '''
    Fit an ellipsoid/sphere to a set of xyz data points:

      [center, radii, evecs, pars ] = ellipsoid_fit( data )
      [center, radii, evecs, pars ] = ellipsoid_fit( np.array([x y z]) );
      [center, radii, evecs, pars ] = ellipsoid_fit( data, option=1 );
      [center, radii, evecs, pars ] = ellipsoid_fit( data, option=2, equals='xz' );
      [center, radii, evecs, pars ] = ellipsoid_fit( data, option=3 );

    Parameters:
    * data=[x y z]    - Cartesian data, n x 3 matrix or three n x 1 vectors
    * option       - 0 fits an arbitrary ellipsoid (default),
                   - 1 fits an ellipsoid with its axes along [x y z] axes
                   - 2 followed by, say, 'xy' fits as 1 but also x_rad = y_rad
                   - 3 fits a sphere
    * equals       - 'xy', 'xz', or 'yz' for option==2

    Output:
    * center    -  ellipsoid center coordinates [xc; yc; zc]
    * ax        -  ellipsoid radii [a; b; c]
    * evecs     -  ellipsoid radii directions as columns of the 3x3 matrix
    * v         -  the 9 parameters describing the ellipsoid algebraically: 
                   Ax^2 + By^2 + Cz^2 + 2Dxy + 2Exz + 2Fyz + 2Gx + 2Hy + 2Iz = 1

    This needs: numpy
    
    Author:
    Yury Petrov, Northeastern University, Boston, MA
    '''

    option  = kwargs.get('option', 0)
    equals  = kwargs.get('equals', 'xy')
    
    if data.shape[1] != 3:
        raise ValueError('Input data must have three columns!')
    else:
        x = data[:, 0]
        y = data[:, 1]
        z = data[:, 2]
    
    # Need nine or more data points
    if len(x) < 9 and option == 0:
        raise ValueError('Must have at least 9 points to fit a unique ellipsoid')
    if len(x) < 6 and option == 1:
        raise ValueError('Must have at least 6 points to fit a unique oriented ellipsoid')
    if len(x) < 5 and option == 2:
        raise ValueError('Must have at least 5 points to fit a unique oriented ellipsoid with two axes equal')
    if len(x) < 3 and option == 3:
        raise ValueError('Must have at least 4 points to fit a unique sphere')
    
    if   option == 0: # Fit ellipsoid in the form Ax^2 + By^2 + Cz^2 + 2Dxy + 2Exz + 2Fyz + 2Gx + 2Hy + 2Iz = 1
        D = np.column_stack((x*x, y*y, z*z, 2*x*y, 2*x*z, 2*y*z, 2*x, 2*y, 2*z))
    elif option == 1: # Fit ellipsoid in the form Ax^2 + By^2 + Cz^2 + 2Gx + 2Hy + 2Iz = 1
        D = np.column_stack((x*x, y*y, z*z, 2*x, 2*y, 2*z))
    elif option == 2: # Fit ellipsoid in the form Ax^2 + By^2 + Cz^2 + 2Gx + 2Hy + 2Iz = 1, where A = B or B = C or A = C
        if equals == 'yz' or equals == 'zy':
            D = np.column_stack((y*y+z*z, x*x, 2*x, 2*y, 2*z))
        elif equals == 'xz' or equals == 'zx':
            D = np.column_stack((x*x+z*z, y*y, 2*x, 2*y, 2*z))
        else:
            D = np.column_stack((x*x+y*y, z*z, 2*x, 2*y, 2*z))
    else:
        # Fit sphere in the form A(x^2 + y^2 + z^2) + 2Gx + 2Hy + 2Iz = 1
        D = np.column_stack((x*x+y*y+z*z, 2*x, 2*y, 2*z))
    
    # Solve the normal system of equations
    v = np.linalg.lstsq(D.T @ D, D.T @ np.ones(len(x)), rcond=None)[0]
    
    # Find the ellipsoid parameters
    if option == 0:
        # Form the algebraic form of the ellipsoid
        A = np.array([[v[0],   v[3], v[4], v[6]],
                        [v[3], v[1], v[5], v[7]],
                        [v[4], v[5], v[2], v[8]],
                        [v[6], v[7], v[8],  -1]])
        
        # Find the center of the ellipsoid
        center = -np.linalg.inv(A[:3, :3]) @ np.array([v[6], v[7], v[8]])
        
        # Form the corresponding translation matrix
        T = np.eye(4)
        T[:3, 3] = center
        
        # Translate to the center
        R = T @ A @ T.T
        
        # Solve the Eigen problem
        evals, evecs = np.linalg.eig(-R[:3, :3] / R[3, 3])
        radii = np.sqrt(np.abs(1. / evals))
    else:
        if option == 1:
            v = np.array([v[0], v[1], v[2], 0, 0, 0, v[3], v[4], v[5]])
        elif option == 2:
            if equals == 'xz' or equals == 'zx':
                v = np.array([v[0], v[1], v[0], 0, 0, 0, v[2], v[3], v[4]])
            elif equals == 'yz' or equals == 'zy':
                v = np.array([v[1], v[0], v[0], 0, 0, 0, v[2], v[3], v[4]])
            else:  # xy
                v = np.array([v[0], v[0], v[1], 0, 0, 0, v[2], v[3], v[4]])
        else:
            v = np.array([v[0], v[0], v[0], 0, 0, 0, v[1], v[2], v[3]])
        
        center = -v[6:9] / v[0:3]
        gam = 1 + (v[6] ** 2 / v[0] + v[7] ** 2 / v[1] + v[8] ** 2 / v[2])
        radii = np.sqrt(gam / v[0:3])
        evecs = np.eye(3)
    
    return center, radii, evecs, v


if __name__ == "__main__":
  
    import matplotlib.pyplot as plt
    from pyIMU.utilities import gravity

    print("Calibrating Gyroscope")
    ####################################################################

    data = readfromFile('0RPMgyr.txt')
    data = cleanupData(data,0,0.1)
    data = removeOutlayers(data)
    data = removeOutlayers(data)
    data = removeOutlayers(data)
    offset = np.average(data, axis=0)

    # fig = plt.figure()
    # ax = fig.add_subplot(projection='3d')
    # ax.scatter(data[:,0], data[:,1], data[:,2], marker='o', alpha=0.5)
    # ax.scatter(offset[0], offset[1], offset[2], marker='^', alpha=1.0)
    # ax.set_xlabel('X')
    # ax.set_ylabel('Y')
    # ax.set_zlabel('Z')
    # plt.show()

    target_norm = 100./3.

    data = readfromFile('33RPMgyr.txt')
    data = cleanupData(data,2.,5.)
    data = removeOutlayers(data)
    data = removeOutlayers(data)
    data = removeOutlayers(data)

    # Correct from previous calculated offset
    data -= offset

    offset_Corr, correctionMat, radii = calibrate(data,1)                                # option 0=full ellipse, 1=ellipse aligned with x,y,z

    # print(radii/np.pi/2.0*60.)
    # print(offset)
    # print(offset_Corr)

    # (ex,ey,ez) = ellipsoid(offset_Corr,radii)
    # fig = plt.figure()
    # ax = fig.add_subplot(projection='3d')
    # ax.scatter(data[:,0], data[:,1], data[:,2], marker='o', alpha=0.25, color='blue')
    # ax.scatter(offset[0], offset[1], offset[2], marker='^', alpha=0.25, color='red')
    # ax.plot_surface(ex,ey,ez, rstride=4, cstride=4, alpha=0.1, color='black')
    # ax.set_xlabel('X')
    # ax.set_ylabel('Y')
    # ax.set_zlabel('Z')
    # plt.show()
    # input("Press Enter to continue...")

    # print(correctionMat)
    radii_Corr = radii @ correctionMat
    # print(radii_Corr/np.pi/2.0*60.)

    s=target_norm/(radii_Corr[0]/np.pi/2.*60.)
    correctionMat *= s

    radii_Corr = radii @ correctionMat
    # print(radii_Corr/np.pi/2.0*60.)

    data_Corr = computeCalibratedData(data, offset_Corr, correctionMat)

    error=target_norm-(np.linalg.norm(data_Corr, axis=1)/np.pi/2.0*60.)

    # fig = plt.figure()
    # ax = fig.add_subplot()
    # ax.plot(error, marker='o')
    # plt.show()

    saveCalibration('33Corr.json',offset,correctionMat)
    correctionMat33 = correctionMat
    ##################################

    target_norm = 45.

    data = readfromFile('45RPMgyr.txt')
    # n = np.linalg.norm(data, axis=1)
    # fig = plt.figure()
    # ax = fig.add_subplot()
    # ax.plot(n, marker='o')
    # plt.show()

    data = cleanupData(data,4.5,5.5)
    data = removeOutlayers(data)
    data = removeOutlayers(data)
    data = removeOutlayers(data)

    # Correct from previous calculated offset
    data -= offset

    offset_Corr, correctionMat, radii = calibrate(data,1)                                # option 0=full ellipse, 1=ellipse aligned with x,y,z

    radii_Corr = radii @ correctionMat
    s=target_norm/(radii_Corr[0]/np.pi/2.*60.)
    correctionMat *= s

    radii_Corr = radii @ correctionMat

    data_Corr = computeCalibratedData(data, offset_Corr, correctionMat)

    error=target_norm-(np.linalg.norm(data_Corr, axis=1)/np.pi/2.0*60.)

    # fig = plt.figure()
    # ax = fig.add_subplot()
    # ax.plot(error, marker='o')
    # plt.show()

    radii/np.pi/2.*60
    radii_Corr/np.pi/2.*60

    saveCalibration('45Corr.json',offset,correctionMat)

    correctionMat44 = correctionMat

    correctionMat = (correctionMat33 + correctionMat44)/2.
    saveCalibration('Gyr.json',offset,correctionMat)

    print("Calibrating Accelerometer")
    ####################################################################

    # Default values are for Tucson, Arizona, USA
    latitude         = 32.253460  # decimal degrees
    altitude         = 730        # meter
    target_norm = gravity(latitude, altitude)

    data = readfromFile('Officeacc.txt')
    # n = np.linalg.norm(data, axis=1)
    # fig = plt.figure()
    # ax = fig.add_subplot()
    # ax.plot(n, marker='o')
    # plt.show()
    data = cleanupData(data,9.5,10.5)
    data = removeOutlayers(data)
    data = removeOutlayers(data)
    data = removeOutlayers(data)
    # n = np.linalg.norm(data, axis=1)
    # fig = plt.figure()
    # ax = fig.add_subplot()
    # ax.plot(n, marker='o')
    # plt.show()

    offset, correctionMat, radii = calibrate(data,1)                                # option 0=full ellipse, 1=ellipse aligned with x,y,z

    print(radii)
    print(offset)

    (ex,ey,ez) = ellipsoid(offset,radii)
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.scatter(data[:,0], data[:,1], data[:,2], marker='o', alpha=0.25, color='blue')
    ax.scatter(offset[0], offset[1], offset[2], marker='^', alpha=0.25, color='red')
    ax.plot_surface(ex,ey,ez, rstride=4, cstride=4, alpha=0.1, color='black')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()

    print(correctionMat)
    radii_Corr = radii @ correctionMat
    print(radii_Corr)

    s=target_norm/(radii_Corr[0])
    correctionMat *= s

    radii_Corr = radii @ correctionMat
    print(radii_Corr)

    data_Corr = computeCalibratedData(data, offset_Corr, correctionMat)

    error=target_norm-(np.linalg.norm(data_Corr, axis=1))

    fig = plt.figure()
    ax = fig.add_subplot()
    ax.plot(error, marker='o')
    plt.show()

    saveCalibration('Acc.json',offset,correctionMat)
    # input("Press Enter to continue...")

    ##################################

    print("Calibrating Magnetometer")
    ####################################################################

    # Default values are for Tucson, Arizona, USA
    target_norm = 47.3923    # microT

    data = readfromFile('Officemag.txt')
    n = np.linalg.norm(data, axis=1)
    fig = plt.figure()
    ax = fig.add_subplot()
    ax.plot(n, marker='o')
    plt.show()
    data = cleanupData(data,0,500)
    data = removeOutlayers(data)
    data = removeOutlayers(data)
    data = removeOutlayers(data)
    n = np.linalg.norm(data, axis=1)
    fig = plt.figure()
    ax = fig.add_subplot()
    ax.plot(n, marker='o')
    plt.show()

    offset, correctionMat, radii = calibrate(data,1)                                # option 0=full ellipse, 1=ellipse aligned with x,y,z

    print(radii)
    print(offset)

    (ex,ey,ez) = ellipsoid(offset,radii)
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.scatter(data[:,0], data[:,1], data[:,2], marker='o', alpha=0.25, color='blue')
    ax.scatter(offset[0], offset[1], offset[2], marker='^', alpha=0.25, color='red')
    ax.plot_surface(ex,ey,ez, rstride=4, cstride=4, alpha=0.1, color='black')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()

    print(correctionMat)
    radii_Corr = radii @ correctionMat
    print(radii_Corr)

    s=target_norm/(radii_Corr[0])
    correctionMat *= s

    radii_Corr = radii @ correctionMat
    print(radii_Corr)

    data_Corr = computeCalibratedData(data, offset_Corr, correctionMat)

    error=target_norm-(np.linalg.norm(data_Corr, axis=1))

    fig = plt.figure()
    ax = fig.add_subplot()
    ax.plot(error, marker='o')
    plt.show()

    saveCalibration('Mag.json',offset,correctionMat)

    ##################################

    # input("Press Enter to continue...")
