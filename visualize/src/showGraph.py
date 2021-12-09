import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure()
ax = fig.add_subplot(111,projection='3d')

def animate(i):

    file = open("data.txt","r")
    graph_data = file.read()
    file.close()

    lines = graph_data.split("\n")
    xs=[]
    ys=[]
    zs=[]

    for line in lines:
        if len(line) > 1 :
            x,y,z = line.split(",")
            xs.append(float(x)) # robots axis
            ys.append(float(y))
            zs.append(float(z))
    

    Xd = np.asarray(xs)
    XC = Xd.reshape(1,len(Xd)).T

    Yd = np.asarray(ys)
    YC = Yd.reshape(1,len(Yd)).T

    Zd = np.asarray(zs)
    ZC = Zd.reshape(1,len(Zd)).T
    
    ax.clear()
    ax.scatter(xs,ys,zs, c = 'red',marker='o')
    # ax.plot(zs,xs,ys, color = 'black')
    ax.plot_wireframe(XC,YC,ZC,edgecolor="limegreen")
    ax.set_xlabel("X direction")
    ax.set_ylabel("Y direction")
    ax.set_zlabel("Z direction")

ani = animation.FuncAnimation(fig,animate,interval = 1000)
plt.show()