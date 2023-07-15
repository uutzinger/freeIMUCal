
import matplotlib.pyplot as plt
from pyIMU.utilities import gravity
from cal_lib import *

print("Calibrating Gyroscope")
####################################################################

# OFFSET FITTING
####################################################################
data = readfromFile('0RPMgyr.txt')
data = cleanupData(data,0,0.1)
data = removeOutliers(data)
data = removeOutliers(data)
data = removeOutliers(data)
offset = np.average(data, axis=0)

n = np.linalg.norm(data, axis=1)
fig = plt.figure()
ax = fig.add_subplot()
ax.plot(n, marker='o')
fig.suptitle("Readings as magnitude")
plt.show()

# RPM 33.3 FITTING
####################################################################

target_norm = 100./3./60.*2.*np.pi

data = readfromFile('33RPMgyr.txt')
data = cleanupData(data,2.,5.)
data = removeOutliers(data)
data = removeOutliers(data)
data = removeOutliers(data)

# Correct from previous calculated offset
data -= offset

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

offset_Corr, correctionMat, radii = calibrate(data,1)                                # option 0=full ellipse, 1=ellipse aligned with x,y,z

print(radii/np.pi/2.0*60.)
print(offset)
print(offset_Corr)

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


saveCalibration('33Corr.json',offset,correctionMat)
correctionMat33 = correctionMat


# RPM 45 FITTING
####################################################################


target_norm = 45./60.*2.*np.pi  # radians per second

data = readfromFile('45RPMgyr.txt')

data -= offset

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

data = cleanupData(data,4.5,5.5)
data = removeOutliers(data)
data = removeOutliers(data)
data = removeOutliers(data)

offset_Corr, correctionMat, radii = calibrate(data,1)                                # option 0=full ellipse, 1=ellipse aligned with x,y,z

print(radii)
print(offset_Corr)

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

radii_Corr = radii @ correctionMat
s=target_norm/(radii_Corr[0])
correctionMat *= s

radii_Corr = radii @ correctionMat

data_Corr = computeCalibratedData(data, offset_Corr, correctionMat)

error=target_norm-(np.linalg.norm(data_Corr, axis=1))

print(radii/np.pi/2.*60)
print(radii_Corr/np.pi/2.*60)

saveCalibration('45Corr.json',offset,correctionMat)

fig = plt.figure()
ax = fig.add_subplot()
ax.plot(error, marker='o')
fig.suptitle("Residuals of fit")
plt.show()

correctionMat45 = correctionMat

correctionMat = (correctionMat33 + correctionMat45)/2.
saveCalibration('Gyr.json',offset,correctionMat)

##################################

input("Press Enter to continue...")
