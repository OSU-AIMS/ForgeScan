import numpy as np
import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


# Function to visualize the data
def visualize_data(data):
    xs = np.array([])
    ys = np.array([])
    zs = np.array([])
    deltaxs = np.array([])
    deltays = np.array([])
    deltazs = np.array([])
    ntimes = np.array([])
    count = 0
    for row in data:
        x, y, z, dx, dy, dz, n_times = row
        xs = np.append(xs,x)
        ys = np.append(ys,y)
        zs = np.append(zs,z)
        deltaxs = np.append(deltaxs,dx)
        deltays = np.append(deltays,dy)
        deltazs = np.append(deltazs,dz)
        ntimes = np.append(ntimes, n_times)
        count = count + 1
        if(count==1000):
            break
    xs = xs.astype('float64')
    ys = ys.astype('float64')
    zs = zs.astype('float64')
    deltaxs = deltaxs.astype('float64')
    deltays = deltays.astype('float64')
    deltazs = deltazs.astype('float64')
    ntimes = ntimes.astype('float64')
    ## Color vectors according to times seen
    fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
    ax.scatter(xs,ys,zs)
    ax.quiver(xs,ys,zs,deltaxs,deltays,deltazs)
    ax.set(xticklabels=[],yticklabels=[],zticklabels=[])

    plt.show()

# Example usage:
with open('share/Images/Vertex_Vector_Data_10.csv', 'r') as f:
    reader = csv.reader(f)
    data = list(reader)
    data_array = np.array(data, dtype=float)
    print(type(data_array[0][0]))
    visualize_data(data)
