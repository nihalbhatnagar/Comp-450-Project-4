import numpy
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from mpl_toolkits.mplot3d import Axes3D

data = numpy.loadtxt('../carPathRRT.txt')
fig = plt.figure()
ax = fig.gca()
ax.add_patch(Rectangle((2, 6), 2, 1))
print(data)
ax.plot(data[:,0],data[:,1],'.-')

plt.show()


data = numpy.loadtxt('../carPathKPIECE1.txt')
fig = plt.figure()
ax = fig.gca()
ax.add_patch(Rectangle((2, 6), 2, 1))
print(data)
ax.plot(data[:,0],data[:,1],'.-')

plt.show()