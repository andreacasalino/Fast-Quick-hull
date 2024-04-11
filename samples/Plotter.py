# -*- coding: utf-8 -*-
"""
* Author:    Andrea Casalino
* Created:   26.12.2019
*
* report any bug to andrecasa91@gmail.com.
"""

from stl import mesh
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.pyplot as plt
from matplotlib.widgets import CheckButtons
from optparse import OptionParser
import json

# https://pypi.org/project/numpy-stl/
class Stl:
    def __init__(self, filename, ax):
        self.mesh = mesh.Mesh.from_file(filename)
        self.hndlr = Poly3DCollection(self.mesh.vectors)
        self.add(ax)

    def add(self, ax):
        ax.add_collection3d(self.hndlr)
        # Auto scale to the mesh size
        scale = self.mesh.points.flatten()
        ax.auto_scale_xyz(scale, scale, scale)

    def remove(self):
        self.hndlr.remove()

class PointCloud:
    def __init__(self, data, ax):        
        self.Vx = []
        self.Vy = []
        self.Vz = []
        for p in data['Cloud']:
            self.Vx.append(p[0])
            self.Vy.append(p[1])
            self.Vz.append(p[2])
        self.add(ax)

    def add(self, ax):
        self.pointcloud = ax.plot(self.Vx, self.Vy, self.Vz, '.r', markersize=10)[0]

    def remove(self):
        self.pointcloud.remove()

class Hull:
    def __init__(self, data, ax):        
        def make_facet(Pa, Pb, Pc):
            x = [Pa[0], Pb[0], Pc[0]]
            y = [Pa[1], Pb[1], Pc[1]]
            z = [Pa[2], Pb[2], Pc[2]]
            return Poly3DCollection([list(zip(x,y,z))], edgecolor='black', facecolors='green', alpha=0.5, linewidth=0.2)

        self.faces = [make_facet(data['Cloud'][index[0]], data['Cloud'][index[1]], data['Cloud'][index[2]]) for index in data['Index']]
            
        self.add(ax)

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

    def add(self, ax):
        for facet in self.faces:
            ax.add_collection3d(facet)

    def remove(self):
        for facet in self.faces:
            facet.remove()

def importConvexHull(filename, ax):
    with open(filename) as stream:
        data = json.load(stream)  
        return (PointCloud(data, ax) , Hull(data, ax))

class Figure:
    def __init__(self):
        self.fig=plt.figure()
        gs = self.fig.add_gridspec(1, 6)
        self.ax1 = self.fig.add_subplot(gs[0, 0])
        self.ax2 = self.fig.add_subplot(gs[0, 1:],projection='3d')
        self.handlers = {}

    def addHandler_(self, label, hndlr):
        self.handlers[label] = {
            'state':True,
            'handler':hndlr
        }

    def addCH(self, filename):
        point_cloud, hull = importConvexHull(filename, self.ax2)
        self.addHandler_('point_cloud', point_cloud)
        self.addHandler_('convex hull', hull)

    def addSTL(self, filename):
        self.addHandler_('stl', Stl(filename, self.ax2))

    def onButtonsClicked_(self, label):
        info = self.handlers[label]
        info['state'] = not info['state']
        if info['state']:
            info['handler'].add(self.ax2)
        else:
            info['handler'].remove()
        self.fig.show()

    def finalize(self):
        labels = tuple(self.handlers.keys())
        flags = tuple([True for _ in self.handlers.keys()])
        self.check = CheckButtons(self.ax1, labels, flags)
        self.ax1.axis('off')
        self.check.on_clicked(lambda label : self.onButtonsClicked_(label))

if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option("--CH", default=None)
    parser.add_option("--STL", default=None)
    (options, args) = parser.parse_args()

    figure = Figure()
    if not options.STL == None:
        figure.addSTL(options.STL)
    if not options.CH == None:
        figure.addCH(options.CH)
    figure.finalize()

    plt.show()
