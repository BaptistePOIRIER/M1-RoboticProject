# -*- coding: utf-8 -*-
"""
Way Point navigtion

(c) S. Bertrand
"""

import math
import Robot as rob
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import Timer as tmr
import Potential

# robot
x0 = -20.0
y0 = -20.0
theta0 = 0.00001
robot = rob.Robot(x0, y0, theta0)


# potential
pot = Potential.Potential(difficulty=1, random=True)


# position control loop: gain and timer
kpPos = 1
positionCtrlPeriod = 0.2#0.01
timerPositionCtrl = tmr.Timer(positionCtrlPeriod)

# orientation control loop: gain and timer
kpOrient = 10
orientationCtrlPeriod = 0.05#0.01
timerOrientationCtrl = tmr.Timer(orientationCtrlPeriod)


# list of way points: list of [x coord, y coord]
WPlist = [[pot.mu1[0],pot.mu1[1]]]
#threshold for change to next WP
epsilonWP = 1
# init WPManager
WPManager = rob.WPManager(WPlist, epsilonWP)

# duration of scenario and time step for numerical integration
t0 = 0.0
tf = 50.0
dt = 0.01
simu = rob.RobotSimulation(robot, t0, tf, dt)


# initialize control inputs
Vr = 0.0
thetar = 0.0
omegar = 0.0

firstIter = True

def move(dist, angle):
    newAngle = robot.theta + angle
    
    newX = dist * np.cos(newAngle)
    newY = dist * np.sin(newAngle)
    
    return [robot.x + newX, robot.y + newY]

potentialValues = [0.0]
STATE = -1
points = []
#points = [[2.0, 1.0, np.sqrt(10)],[5.0, 4.0, 2.0],[8.0, 2.0, 4.0]]
potentialMax = 313.201823198

# loop on simulation time
for t in simu.t: 
   


    # position control loop
    if timerPositionCtrl.isEllapsed(t):

        potentialValue = pot.value([robot.x, robot.y])
        potentialValues.append(potentialValue)

        # algo
        if (STATE == -1):
            aim = move(2.0,0.0)
            if (potentialValue > 0):
                points.append([robot.x,robot.y,potentialValue])
                STATE = 0
                
        elif (STATE == 0):
            aim = move(2.0,1.0)
            if(len(points) < 3):
                if(np.sqrt((points[-1][0]-robot.x)**2+(points[-1][1]-robot.y)**2) < 1):
                    points.append([robot.x,robot.y,abs(potentialMax-potentialValue)])
            else:
                print(points)
                STATE = 1
        elif (STATE == 1):
            a = 2*(points[2][0]-points[0][0])
            b = 2*(points[2][1]-points[0][1])
            c = 2*(points[2][0]-points[1][0])
            d = 2*(points[2][1]-points[1][1])
            matrixDeterminant = a * b - (c * d)
            if(matrixDeterminant == 0):
                points = []
                STATE == 0
            else:
                a_ = (1/matrixDeterminant)*d
                b_ = (1/matrixDeterminant)*(-b)
                c_ = (1/matrixDeterminant)*(-c)
                d_ = (1/matrixDeterminant)*a
                x = a_ * (points[0][2]**2 - points[2][2]**2 + points[2][0]**2 - points[0][0]**2 + points[2][1]**2 - points[0][1]**2)
                x += b_ * (points[1][2]**2 - points[2][2]**2 + points[2][0]**2 - points[1][0]**2 + points[2][1]**2 - points[1][1]**2)
                y = c_ * (points[0][2]**2 - points[2][2]**2 + points[2][0]**2 - points[0][0]**2 + points[2][1]**2 - points[0][1]**2)
                y += d_ * (points[1][2]**2 - points[2][2]**2 + points[2][0]**2 - points[1][0]**2 + points[2][1]**2 - points[1][1]**2)
                aim = [x,y]
                print(x,y)
                STATE = 2
            
        elif (STATE == 2):
            if (Vr < 1):
                print(potentialValue)
                STATE = 3
        
        # velocity control input
        Vr = kpPos * np.sqrt((aim[0] - robot.x)**2 + (aim[1] - robot.y)**2)
        
        
        # reference orientation
        thetar = np.arctan2(aim[1] - robot.y,aim[0] - robot.x)
        
        
        if math.fabs(robot.theta-thetar)>math.pi:
            thetar = thetar + math.copysign(2*math.pi,robot.theta)        
        
        
        
    # orientation control loop
    if timerOrientationCtrl.isEllapsed(t):
        # angular velocity control input        
        omegar = kpOrient * (thetar - robot.theta)
    
    
    # assign control inputs to robot
    robot.setV(Vr)
    robot.setOmega(omegar)    
    
    # integrate motion
    robot.integrateMotion(dt)

    # store data to be plotted   
    simu.addData(robot, WPManager, Vr, thetar, omegar, pot.value([robot.x,robot.y]))
    
    
# end of loop on simulation time


# close all figures
plt.close("all")

# generate plots
fig,ax = simu.plotXY(1)
pot.plot(noFigure=None, fig=fig, ax=ax)  # plot potential for verification of solution

#simu.plotXYTheta(2)
#simu.plotVOmega(3)

#simu.plotPotential(4)



#simu.plotPotential3D(5)


# show plots
#plt.show()





# # Animation *********************************
#fig = plt.figure()
#ax = fig.add_subplot(111, aspect='equal', autoscale_on=False, xlim=(-25, 25), ylim=(-25, 25))
#ax.grid()
#ax.set_xlabel('x (m)')
#ax.set_ylabel('y (m)')
#
#robotBody, = ax.plot([], [], 'o-', lw=2)
#robotDirection, = ax.plot([], [], '-', lw=1, color='k')
#wayPoint, = ax.plot([], [], 'o-', lw=2, color='b')
#time_template = 'time = %.1fs'
#time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)
#potential_template = 'potential = %.1f'
#potential_text = ax.text(0.05, 0.1, '', transform=ax.transAxes)
#WPArea, = ax.plot([], [], ':', lw=1, color='b')
#
#thetaWPArea = np.arange(0.0,2.0*math.pi+2*math.pi/30.0, 2.0*math.pi/30.0)
#xWPArea = WPManager.epsilonWP*np.cos(thetaWPArea)
#yWPArea = WPManager.epsilonWP*np.sin(thetaWPArea)
#
#def initAnimation():
#    robotDirection.set_data([], [])
#    robotBody.set_data([], [])
#    wayPoint.set_data([], [])
#    WPArea.set_data([], [])
#    robotBody.set_color('r')
#    robotBody.set_markersize(20)    
#    time_text.set_text('')
#    potential_text.set_text('')
#    return robotBody,robotDirection, wayPoint, time_text, potential_text, WPArea  
#
#def animate(i):  
#    robotBody.set_data(simu.x[i], simu.y[i])          
#    wayPoint.set_data(simu.xr[i], simu.yr[i])
#    WPArea.set_data(simu.xr[i]+xWPArea.transpose(), simu.yr[i]+yWPArea.transpose())    
#    thisx = [simu.x[i], simu.x[i] + 0.5*math.cos(simu.theta[i])]
#    thisy = [simu.y[i], simu.y[i] + 0.5*math.sin(simu.theta[i])]
#    robotDirection.set_data(thisx, thisy)
#    time_text.set_text(time_template%(i*simu.dt))
#    potential_text.set_text(potential_template%(pot.value([simu.x[i],simu.y[i]])))
#    return robotBody,robotDirection, wayPoint, time_text, potential_text, WPArea
#
#ani = animation.FuncAnimation(fig, animate, np.arange(1, len(simu.t)),
#                              interval=4, blit=True, init_func=initAnimation, repeat=False)
# #interval=25

# #ani.save('robot.mp4', fps=15)

