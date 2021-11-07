# -*- coding: utf-8 -*-
"""
Created on Thu Mar 18 23:10:16 2021

(c) S. Bertrand
"""


import numpy as np

import matplotlib.pyplot as plt

from scipy.stats import multivariate_normal



class Potential:

    def __init__(self, difficulty=1, random=False):
        
        if (difficulty<1)or(difficulty>3):
            raise NameError("Difficulty must be >=1 and <=3")
        
        self.difficulty = difficulty
        self.random = random
        
        self.xmin = -50.
        self.xmax = 50.
        self.xstep = 0.05
        self.ymin = -50.
        self.ymax = 50.
        self.ystep = 0.05
        
        
        
        if (random):
            xwidth = np.abs(50)
            ywidth = np.abs(50)
            self.mu1 = [ 0.6*(xwidth*np.random.rand()-xwidth/2.) , 0.6*(ywidth*np.random.rand()-ywidth/2.) ]
            self.mu2 = [ xwidth*np.random.rand()-xwidth/2. , ywidth*np.random.rand()-ywidth/2. ]
            self.mu3 = [ xwidth*np.random.rand()-xwidth/2. , ywidth*np.random.rand()-ywidth/2. ]
        else:
            self.mu1 = [6, 4]
            self.mu2 = [-2, -2]
            self.mu3 = [-7, 10]
        
        self.gaussian1 = multivariate_normal(self.mu1, [[1.0, 0.], [0., 1.]])
        self.gaussian2 = multivariate_normal(self.mu2, [[0.5, 0.3], [0.3, 0.5]])
        self.gaussian3 = multivariate_normal(self.mu3, [[0.8, 0.], [0., 0.8]])
        
        self.weight1 = 10000
        self.weight2 = 1
        self.weight3 = 1E-8
        
        
        self.mu = [self.mu1]
        if (difficulty>1):
            self.mu.append(self.mu2)
            if (difficulty>2):
                self.mu.append(self.mu3)
        
        self.distribution = [self.gaussian1]
        if (difficulty>1):
            self.distribution.append(self.gaussian2)
            if (difficulty>2):
                self.distribution.append(self.gaussian3)
                
        self.weight = [self.weight1]
        if (difficulty>1):
            self.weight.append(self.weight2)
            if (difficulty>2):
                self.weight.append(self.weight3)


    
    
    def value(self,pos):  # (pos = [x,y]) 
        
        sumval = 0.
        
        for i in range(self.difficulty):
            sumval += self.weight[i]*self.distribution[i].pdf(pos)
        
        return np.fmax(310.+np.log10(sumval), -10.)



    def plot(self,noFigure=None,fig=None,ax=None):

        x, y = np.mgrid[self.xmin:self.xmax:self.xstep, self.ymin:self.ymax:self.ystep]
        pos = np.dstack((x, y))
        potentialFieldForPlot = self.value(pos)
        
        if (fig==None):
            if (noFigure==None):
                noFigure=1
            fig = plt.figure(noFigure)
        if (ax==None):
            ax = fig.add_subplot(111)
        cs = ax.contourf(x, y, potentialFieldForPlot, 20, cmap='BrBG')
        
        fig.colorbar(cs)
        
        return fig, ax






# main
if __name__=='__main__':

    
    plt.close()
    
    pot = Potential(difficulty=1, random=False)
       
    fig2, ax2 = pot.plot(1)
#    points = [[-14,-14],[-8,-12],[-7,-5]]
    center = [6,4]
#    plt.plot(points[0][0], points[0][1], 'bo--', linewidth=2, markersize=4)
#    plt.plot(points[1][0], points[1][1], 'wo--', linewidth=2, markersize=4)
#    plt.plot(points[2][0], points[2][1], 'ro--', linewidth=2, markersize=4)
#    circles = [[],[],[]]
#    density = 500
#    for i in range(3):
#        dist = np.sqrt((points[i][0]-center[0])**2+(points[i][1]-center[1])**2)
#        for j in range(density):
#            circles[i].append([points[i][0]+(np.cos(j*2*np.pi/density))*dist,points[i][1]+(np.sin(j*2*np.pi/density))*dist])
#
#    for point in circles[0]:
#        plt.plot(point[0], point[1], 'bo--', linewidth=2, markersize=1)
#    for point in circles[1]:
#        plt.plot(point[0], point[1], 'wo--', linewidth=2, markersize=1)
#    for point in circles[2]:
#        plt.plot(point[0], point[1], 'ro--', linewidth=2, markersize=1)
#
#    plt.plot(center[0], center[1], 'go--', linewidth=2, markersize=10)
    
    tangentes = [[6.3999999999793173, -19.999967000001959, 1.2500000003970923e-06], [16.223467124991835, -15.059861549008129, 0.46875125000000017]]
    plt.plot(tangentes[0][0], tangentes[0][1], 'bo--', linewidth=2, markersize=4)
    plt.plot(tangentes[1][0], tangentes[1][1], 'ro--', linewidth=2, markersize=4)
    
    length = 30
    size = 1
    for i in range(length):
        plt.plot(tangentes[0][0]+(np.cos(tangentes[0][2])*i*size), tangentes[0][1]+(np.sin(tangentes[0][2])*i*size), 'bo--', linewidth=2, markersize=1)
        plt.plot(tangentes[0][0]+(np.cos(tangentes[0][2])*-i*size), tangentes[0][1]+(np.sin(tangentes[0][2])*-i*size), 'bo--', linewidth=2, markersize=1)
        
    for i in range(length):
        plt.plot(tangentes[1][0]+(np.cos(tangentes[1][2])*i*size), tangentes[1][1]+(np.sin(tangentes[1][2])*i*size), 'ro--', linewidth=2, markersize=1)
        plt.plot(tangentes[1][0]+(np.cos(tangentes[1][2])*-i*size), tangentes[1][1]+(np.sin(tangentes[1][2])*-i*size), 'ro--', linewidth=2, markersize=1)
    
    length = 60
    size = 0.5
    
    for i in range(length):
        plt.plot(tangentes[0][0]+(np.cos(tangentes[0][2]+np.pi/2)*i*size), tangentes[0][1]+(np.sin(tangentes[0][2]+np.pi/2)*i*size), 'bo--', linewidth=2, markersize=2)
        plt.plot(tangentes[1][0]+(np.cos(tangentes[1][2]-np.pi/2)*-i*size), tangentes[1][1]+(np.sin(tangentes[1][2]-np.pi/2)*-i*size), 'ro--', linewidth=2, markersize=2)
        
    plt.plot(center[0], center[1], 'go--', linewidth=2, markersize=10)
    print(pot.value( [6. , 4.] ))
    print(pot.value( [-2., 2.]) )

    
    plt.show()
