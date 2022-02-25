# -*- coding: utf-8 -*-
"""
* Author:    Andrea Casalino
* Created:   26.12.2019
*
* report any bug to andrecasa91@gmail.com.
"""

from stl import mesh
from mpl_toolkits import mplot3d
from matplotlib import pyplot

from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.pyplot as plt
import json
import sys



def get_json_from_file(name):
    with open(name) as json_file:
        return json.load(json_file)
    
def plot_facet(Pa, Pb, Pc, ax, col, alp):
    x = [Pa[0], Pb[0], Pc[0]]
    y = [Pa[1], Pb[1], Pc[1]]
    z = [Pa[2], Pb[2], Pc[2]]
    ax.add_collection3d(Poly3DCollection([list(zip(x,y,z))], edgecolor='black', facecolors=col, alpha=alp, linewidth=0.2))

def plot_CH(file, color, ax):
    data = get_json_from_file(file)
    
    Vx = []
    Vy = []
    Vz = []
    V = data['Cloud']
    for i in V:
        Vx.append(i[0])
        Vy.append(i[1])
        Vz.append(i[2])
    ax.plot(Vx ,Vy ,Vz , '.r')
    
    for index in data['Index']:
        plot_facet(data['Cloud'][index[0]], data['Cloud'][index[1]], data['Cloud'][index[2]], ax, color, 0.5)
        
    x_lim = [0,0]
    y_lim = [0,0]
    z_lim = [0,0]
    for c in data['Cloud']:
        if(c[0] < x_lim[0]): 
            x_lim[0] = c[0]
        if(c[0] > x_lim[1]): 
            x_lim[1] = c[0]
        
        if(c[1] < y_lim[0]):
            y_lim[0] = c[1]
        if(c[1] > y_lim[1]):
            y_lim[1] = c[1]

        if(c[2] < z_lim[0]):
            z_lim[0] = c[2]
        if(c[2] > z_lim[1]):
            z_lim[1] = c[2]
            
    lim = [min(x_lim[0],y_lim[0],z_lim[0]), max(x_lim[1],y_lim[1],z_lim[1])]        
                        
    ax.set_xlim3d(lim[0], lim[1])
    ax.set_ylim3d(lim[0], lim[1])
    ax.set_zlim3d(lim[0], lim[1])

def plot_STL(file, ax):
    # Load the STL files and add the vectors to the plot
    your_mesh = mesh.Mesh.from_file(file)
    ax.add_collection3d(mplot3d.art3d.Poly3DCollection(your_mesh.vectors))
    # Auto scale to the mesh size
    scale = your_mesh.points.flatten()
    ax.auto_scale_xyz(scale, scale, scale)

logName = sys.argv[1]
stlLocation = sys.argv[2]
fig = plt.figure()
ax = fig.gca(projection='3d')
plot_CH(logName, 'green', ax)
plot_STL(stlLocation, ax)
plt.show()
