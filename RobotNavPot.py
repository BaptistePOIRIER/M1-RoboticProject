# ========================================================================== #
#                   IMPORTATION DES LIBRAIRES
# ========================================================================== #
import math
import Robot as rob
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import Timer as tmr
import Potential

# ========================================================================== #
#                   VARIABLES DE BASES
# ========================================================================== #
# robot
x0 = -40.0
y0 = -40.0
theta0 = 0.0
robot = rob.Robot(x0, y0, theta0)

# potential
difficulty = 3
pot = Potential.Potential(difficulty=difficulty, random=True)

# position control loop: gain and timer
kpPos = 1
positionCtrlPeriod = 0.2#0.01
timerPositionCtrl = tmr.Timer(positionCtrlPeriod)

# orientation control loop: gain and timer
kpOrient = 10
orientationCtrlPeriod = 0.05#0.01
timerOrientationCtrl = tmr.Timer(orientationCtrlPeriod)

# duration of scenario and time step for numerical integration
t0 = 0.0
tf = 500.0
dt = 0.01
simu = rob.RobotSimulation(robot, t0, tf, dt)

# initialize control inputs
Vr = 0.0
thetar = 0.0
omegar = 0.0

firstIter = True


# ========================================================================== #
#                   FONCTIONS
# ========================================================================== #
"""
Calcul du nouveau point objectif.
"""
def move(dist, angle):
    newAngle = robot.theta + angle
    
    newX = dist * np.cos(newAngle) + robot.x
    newY = dist * np.sin(newAngle) + robot.y
    
    return [newX, newY]

# ========================================================================== #
#                   INITIALISATION DES VARIABLES
# ========================================================================== #
#potentialValues = [0.0]
potentialValues = {"variation":0.0,"current":0.0,"previous":0.0}
STATE = "INIT"
outsidePoints = []
potentialPoints = []
predictionPoint = []

seuil = 150.0
epsilonSeuil = 0.2
epsilonStartSeuil = 4
fullLoopDetectDistance = 5
distanceBetweenPoints = 1
maxValue = 0.0
maxPoint = []
maxCounter = 0

# ========================================================================== #
#                   BOUCLE DE SIMULATION
# ========================================================================== #
# loop on simulation time
for t in simu.t: 

    # position control loop
    if timerPositionCtrl.isEllapsed(t):

        """
        Stoquage du potentiel du robot.
        On stoque ici 3 valeurs :
            - variation :  la valeur de différence de pollution avec l'ittération précédente
            - current :    la valeur actuelle de pollution (utilisé pour calculer la variation)
        """
        potentialValue = pot.value([robot.x, robot.y])
        #potentialValues.append(potentialValue)
        potentialValues = {"variation": potentialValue-potentialValues["current"],
                           "current":   potentialValue}

# ========================================================================== #
#                   AUTOMATE
# ========================================================================== #
        """
        Etape 1 :
        Copier le truc du rapport
        """
        if (STATE == "INIT"):
            if(abs(seuil - potentialValue) < epsilonStartSeuil):
                STATE = "FOLLOWER"

            elif ((potentialValue < seuil and potentialValues["variation"] < 0) or (potentialValue > seuil and potentialValues["variation"] > 0)):
                aim = move(2.0,1.0)

            else:
                aim = move(5.0,0.0)

            """
            Etape 2 :
                Copier le truc du rapport
            """
        elif (STATE == "FOLLOWER"):
            # On est sur le seuil (à epsilonSeuil près)
            if(abs(seuil - potentialValue) < epsilonSeuil):
                aim = move(1.0,0.0)
                # Détection tour complet
                if (len(outsidePoints) > 10 and np.sqrt((outsidePoints[0][0] - robot.x)**2 + (outsidePoints[0][1] - robot.y)**2) < fullLoopDetectDistance):
                    STATE = "COMPUTE"
                # Ajout de points pour le calcul
                elif (abs(seuil - potentialValue) < 0.05):
                    if(len(outsidePoints) == 0):
                        outsidePoints.append([robot.x,robot.y])
                    elif (np.sqrt((outsidePoints[-1][0] - robot.x)**2 + (outsidePoints[-1][1] - robot.y)**2) > distanceBetweenPoints):
                        outsidePoints.append([robot.x,robot.y])
                        
            # Dans le cluster (selon la valeur seuil)
            elif(seuil - potentialValue < 0): # Inside cluster
                if(potentialValues["variation"] > 0):
                    aim = move(0.5,-0.1)
                else:
                    aim = move(0.5,0.0)
                    
            # En dehors du cluster  (selon la valeur seuil)
            else:
                if(potentialValues["variation"] < 0):
                    aim = move(0.5,0.1)
                else:
                    aim = move(0.5,0.0)

            """
            Etape 3 :
                Copier le truc du rapport
            """
        elif (STATE == "COMPUTE"):
            print("Nombre de points extérieurs trouvés : %s en t=%s" %(len(outsidePoints),t))
            for i in range(len(outsidePoints)-2):
                theta1 = np.arctan2((outsidePoints[i][1]-outsidePoints[i+1][1]),(outsidePoints[i][0]-outsidePoints[i+1][0]))
                theta2 = np.arctan2((outsidePoints[i+1][1]-outsidePoints[i+2][1]),(outsidePoints[i+1][0]-outsidePoints[i+2][0]))
                
                a1 = np.sin(theta1+np.pi/2)/np.cos(theta1+np.pi/2)
                a2 = np.sin(theta2+np.pi/2)/np.cos(theta2+np.pi/2)
                
                center1X = (outsidePoints[i][0] + outsidePoints[i+1][0]) / 2 
                center1Y = (outsidePoints[i][1] + outsidePoints[i+1][1]) / 2 
                center2X = (outsidePoints[i+1][0] + outsidePoints[i+2][0]) / 2 
                center2Y = (outsidePoints[i+1][1] + outsidePoints[i+2][1]) / 2 
                
                b1 = center1Y - a1 * center1X
                b2 = center2Y - a2 * center2X
                
                x = (b2 - b1) / (a1 - a2)
                y = a1 * x + b1
                
                if (abs(x) < 25 and abs(y) < 25):
                    potentialPoints.append([x,y])

            print("Nombres de points potentiels calculés :",len(potentialPoints))
            clusters = []
            clustersAmount = min(difficulty,2)
            for i in range(1,clustersAmount+1):
                clusters.append({"center":potentialPoints[i*(len(potentialPoints)//clustersAmount)-1],"points":[]})
            
            centersNotFound = True
            
            while(centersNotFound):
                print("Itterate")
                for cluster in clusters:
                    cluster["points"] = []
                    
                for point in potentialPoints:
                    dist_min = 1000
                    k = 0
                    for i in range(clustersAmount):
                        dist = np.sqrt((clusters[i]["center"][0] - point[0])**2 + (clusters[i]["center"][1] - point[1])**2)
                        if (dist < dist_min):
                            k = i
                            dist_min = dist
                    clusters[k]["points"].append(point)
                
                centersNotFound = False
                for cluster in clusters:
                    center = [0.0,0.0]
                    for point in cluster["points"]:
                        center[0] += point[0]
                        center[1] += point[1]
                    center = [center[0]/len(cluster["points"]),center[1]/len(cluster["points"])]
                    
                    if (cluster["center"] != center):
                        cluster["center"] = center
                        centersNotFound = True
            
            amount = 0
            for cluster in clusters:
                if (amount < len(cluster["points"])):
                    amount = len(cluster["points"])
                    predictionPoint = cluster["center"]
            
            aim = predictionPoint
            STATE = "REACH_CLUSTER"
            
            """
            Etape 4 :
                Copier le truc du rapport
            """
        elif (STATE == "REACH_CLUSTER"):
            if (np.sqrt((predictionPoint[0] - robot.x)**2 + (predictionPoint[1] - robot.y)**2) < 1):
                STATE = "SEARCH_MAX"

            """
            Etape 5 :
                Copier le truc du rapport
            """
        elif (STATE == "SEARCH_MAX"):
            if(potentialValues["variation"] > 0):
                aim = move(0.5,0.0)
                if (maxValue < potentialValue):
                    maxValue = potentialValue
                    maxPoint = [robot.x,robot.y]
                    maxCounter = 0
                else:
                    maxCounter += 1
                    if(maxCounter > 5):
                        aim = maxPoint
                        STATE = "REACH_MAX"
            else:
                aim = move(0.1,1.0)

            """
            Etape 6 :
                Copier le truc du rapport
            """
        elif (STATE == "REACH_MAX"):
            #print(centers)
            if (Vr < 0.1):
                print("Valeur du potentiel final : ",potentialValue)
                print("Valeur du potentiel maximum :",pot.value([pot.mu1[0],pot.mu1[1]]))
                STATE = "END"

# ========================================================================== #
#                   COMMAANDE DU ROBOT
# ========================================================================== #
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

# ========================================================================== #
#                   AFFICHAGE
# ========================================================================== #
# close all figures
plt.close("all")

# generate plots
fig,ax = simu.plotXY(1,-50,50,-50,50)
pot.plot(noFigure=None, fig=fig, ax=ax)  # plot potential for verification of solution
for point in outsidePoints:
    plt.plot(point[0], point[1], 'bo--', linewidth=2, markersize=2)
for point in potentialPoints:
    plt.plot(point[0], point[1], 'ro--', linewidth=2, markersize=2)
plt.plot(predictionPoint[0], predictionPoint[1], 'go--', linewidth=2, markersize=10)
for cluster in clusters:
    plt.plot(cluster["center"][0], cluster["center"][1], 'wo--', linewidth=2, markersize=5)
plt.plot(pot.mu1[0],pot.mu1[1], 'bo--', linewidth=2, markersize=3)
plt.plot(robot.x,robot.y, 'go--', linewidth=2, markersize=3)

#simu.plotXYTheta(2)
#simu.plotVOmega(3)

#simu.plotPotential(4)



#simu.plotPotential3D(5)


# show plots
#plt.show()





# # Animation *********************************
##fig = plt.figure()
#ax = fig.add_subplot(111, aspect='equal', autoscale_on=False, xlim=(-50, 50), ylim=(-50, 50))
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
#                              interval=1, blit=True, init_func=initAnimation, repeat=False)
## #interval=25
#
## #ani.save('robot.mp4', fps=15)
#
