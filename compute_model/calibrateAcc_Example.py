####################################################################
# Example of accelerometer calibration
####################################################################

import matplotlib.pyplot as plt
from pyIMU.utilities import gravity
from cal_lib import *

print("Calibrating Accelerometer")

# Default values are for Tucson, Arizona, USA
latitude         = 32.253460  # decimal degrees
altitude         = 730        # meter
target_norm = gravity(latitude, altitude)

data = readfromFile('acc_data.txt')
n = np.linalg.norm(data, axis=1)
fig = plt.figure()
ax = fig.add_subplot()
ax.plot(n, marker='o')
fig.suptitle("Readings as magnitude")
plt.show()

fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.scatter(data[0:-1:10,0], data[0:-1:10,1], data[0:-1:10,2], marker='o', alpha=0.25, color='blue')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
fig.suptitle("Readings in 3D")
plt.show()

data = cleanupData(data,9.,11.)
data = removeOutliers(data)
data = removeOutliers(data)
data = removeOutliers(data)
n = np.linalg.norm(data, axis=1)
fig = plt.figure()
ax = fig.add_subplot()
ax.plot(n, marker='o')
fig.suptitle("Readings outlier removed")
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
fig.suptitle("Readings 3D with Ellipsoid")
plt.show()

print(correctionMat)
radii_corr = radii @ correctionMat
print(radii_corr)

s=target_norm/(radii_corr[0])
correctionMat *= s

radii_corr = radii @ correctionMat
print(radii_corr)

data_Corr = computeCalibratedData(data, offset, correctionMat)

error=target_norm-(np.linalg.norm(data_Corr, axis=1))

fig = plt.figure()
ax = fig.add_subplot()
ax.plot(error, marker='o')
fig.suptitle("Residuals of fit")
plt.show()

acceptable = np.logical_and(error>-0.1, error<0.1)
error=target_norm-(np.linalg.norm(data_Corr, axis=1))


# saveCalibration('acc_data.json',offset,correctionMat)

####
# 2nd stage
#########################################################



data = data[acceptable,:] 
n = np.linalg.norm(data, axis=1)
fig = plt.figure()
ax = fig.add_subplot()
ax.plot(n, marker='o')
fig.suptitle("Readings 2nd stage outlier removed")
plt.show()

offset, correctionMat, radii = calibrate(data,1)    # option 0=full ellipse, 1=ellipse aligned with x,y,z

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
fig.suptitle("Readings 2nd stage 3D with Ellipsoid")
plt.show()

print(correctionMat)
radii_corr = radii @ correctionMat
print(radii_corr)

s=target_norm/(radii_corr[0])
correctionMat *= s

radii_corr = radii @ correctionMat
print(radii_corr)

data_Corr = computeCalibratedData(data, offset, correctionMat)

error=target_norm-(np.linalg.norm(data_Corr, axis=1))

fig = plt.figure()
ax = fig.add_subplot()
ax.plot(error, marker='o')
fig.suptitle("Residuals of fit")
plt.show()

saveCalibration('acc_data.json',offset,correctionMat)


input("Press Enter to continue...")

