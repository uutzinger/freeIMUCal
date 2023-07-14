####################################################################
# Example of magnetometer calibration
####################################################################

import matplotlib.pyplot as plt
from cal_lib import *

print("Calibrating Magnetometer")

# Default values are for Tucson, Arizona, USA
target_norm = 33.07893064435485     # microT

data = readfromFile('magtest.txt')
n = np.linalg.norm(data, axis=1)
fig = plt.figure()
ax = fig.add_subplot()
ax.plot(n[0:-1:10], marker='o')
plt.show()

input("Press Enter to continue...")

data = cleanupData(data,0,500)
data = removeOutlayers(data)
data = removeOutlayers(data)
data = removeOutlayers(data)
n = np.linalg.norm(data, axis=1)
fig = plt.figure()
ax = fig.add_subplot()
ax.plot(n[0:-1:10], marker='o')
plt.show()

input("Press Enter to continue...")

offset, correctionMat, radii = calibrate(data,1)                                # option 0=full ellipse, 1=ellipse aligned with x,y,z

print(radii)
print(offset)

(ex,ey,ez) = ellipsoid(offset,radii)
fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.scatter(data[0:-1:10,0], data[0:-1:10,1], data[0:-1:10,2], marker='o', alpha=0.25, color='blue')
ax.scatter(offset[0], offset[1], offset[2], marker='^', alpha=0.25, color='red')
ax.plot_surface(ex,ey,ez, rstride=4, cstride=4, alpha=0.1, color='black')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()

input("Press Enter to continue...")

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
ax.plot(error[0:-1:10], marker='o')
plt.show()

saveCalibration('magtest.json',offset,correctionMat)

input("Press Enter to continue...")

