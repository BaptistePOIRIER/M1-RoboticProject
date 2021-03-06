"""
Projet Introduction à la robotique
FORDANT - GOUDIN - POIRIER

Méthode des Missions Inversées
"""

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
"""
Constantes globales pour la gestion des différentes données nécessaires
aux calculs et détermination de la source de pollution.
"""
potentialValues = {"variation":0.0,"current":0.0,"previous":0.0}
STATE = "Atteint Seuil"
outsidePoints = []
potentialPoints = []
predictionPoint = []
maxValue = 0.0
maxPoint = []
maxCounter = 0

"""
Paramètres sur lesquels influer pour changer le comportement de la
cartographie et de la recherche de la source par le calcul.
"""
seuil = 150.0
epsilonSeuil = 0.2
epsilonStartSeuil = 4
fullLoopDetectDistance = 5
distanceBetweenPoints = 1

# ========================================================================== #
#                   BOUCLE DE SIMULATION
# ========================================================================== #
print("==================================================")
print("Démarrage d'une nouvelle simulation")
print(f"Difficulté utilisé : {difficulty}")

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
        potentialValues = {"variation": potentialValue-potentialValues["current"],
                           "current":   potentialValue}

# ========================================================================== #
#                   AUTOMATE
# ========================================================================== #
        """
        Etape 1 :
        Pendant cette étape, le robot se déplace de manière à atteindre la valeur seuil
        qu'il va devoir suivre par la suite. Il s'oriente donc intelligemment pour atteindre
        cette valeur en tournant lorsque c'est nécessaire.
        """
        if (STATE == "Atteint Seuil"):
            if(abs(seuil - potentialValue) < epsilonStartSeuil):
                STATE = "Suivi"

            elif ((potentialValue < seuil and potentialValues["variation"] < 0) or (potentialValue > seuil and potentialValues["variation"] > 0)):
                aim = move(2.0,1.0)

            else:
                aim = move(5.0,0.0)

            """
            Etape 2 :
            Afin de réaliser la cartographie du nuage de pollution, le robot est amené a suivre
            un seuil de pollution prédéfini dans le sens anti-horaire. Pour le suivre efficacement, 
            nous déterminons deux zones :
                - intérieur : le potentiel est plus grand que seuil+ε, le robot tourne à droite si ∆P > 0, sinon il vas tout droit
                - extérieur : le potentiel est plus petit que seuil-ε, le robot tourne à gauchesi ∆P < 0, sinon il vas tout droit
            """
        elif (STATE == "Suivi"):
            # On est sur le seuil (à epsilonSeuil près)
            if(abs(seuil - potentialValue) < epsilonSeuil):
                aim = move(1.0,0.0)
                # Détection tour complet
                if (len(outsidePoints) > 10 and np.sqrt((outsidePoints[0][0] - robot.x)**2 + (outsidePoints[0][1] - robot.y)**2) < fullLoopDetectDistance):
                    STATE = "Calcul"
                
                    """
                    On ajoute les points qui sont espacé d'au moins distanceBetweenPoints et qui on une précision
                    à 0.05 de la valeur seuil.
                    """
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
            Une fois l'étape de cartographie finie, on utilise la méthode des cordes 
            pour obtenir des prédictions sur les centres des clusters.
            On applique ensuite l'algorithme de clustering (ou algorithme des centres mobiles) sur notre tableau de mesures.
            Après avoir récupéré les centres approchés des clusters, nous déterminons le cluster le plus important (qui a le
            plus de points associés)
            """
        elif (STATE == "Calcul"):
            print("Nombre de points extérieurs trouvés : %s en t=%ss" %(len(outsidePoints),t))
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

            print("Nombres de centres potentiels de clusters calculés :",len(potentialPoints))
            
            
            """
            Initialisation des cluster pour l'algorithme des centres mobiles.
            Le nombre de cluster est limité à 2 car on ne prend pas en compte les ellipses.
            On remarque aussi qu'on prend des points les plus loin possible les uns
            des autres pour réduire le nombre d'ittérations de l'algorithme.
            """
            clusters = []
            clustersAmount = min(difficulty,2)
            for i in range(1,clustersAmount+1):
                clusters.append({"center":potentialPoints[i*(len(potentialPoints)//clustersAmount)-1],"points":[]})
            
            """
            Début de l'algorithme des centres mobiles
            """
            print("Démarrage de l'algorithme des centres mobiles...")
            centersNotFound = True
            
            while(centersNotFound):
                print("\tNouvelle ittération...")
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
            
            """
            Choix du meilleur cluster
            """
            amount = 0
            for cluster in clusters:
                if (amount < len(cluster["points"])):
                    amount = len(cluster["points"])
                    predictionPoint = cluster["center"]

            print("Cluster trouvé aux coordonnées : ",predictionPoint)
            aim = predictionPoint
            STATE = "Atteint cluster"
            
            """
            Etape 4 :
            Durant cette étape, le robot va simplement jusqu'à la prédiction calculé précédemment.
            On considère qu'il l'a atteint quand il est à une distance d < 1.
            """
        elif (STATE == "Atteint cluster"):
            if (np.sqrt((predictionPoint[0] - robot.x)**2 + (predictionPoint[1] - robot.y)**2) < 1):
                STATE = "Recherche approfondie"

            """
            Etape 5 :
            Cette étape sert à approfondir la précision de la détection. 
            La fonction consiste à se rapprocher de plus de plus en plus du potentiel maximum 
            jusqu'à avoir une précision extrêmement élevée.
            """
        elif (STATE == "Recherche approfondie"):
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
                        STATE = "Atteint max"
            else:
                aim = move(0.1,1.0)

            """
            Etape 6 :
            On attend ici l'atteinte du maximum trouvé lors de l'étape précédente.
            On considère qu'on l'a atteint quand le robot à une vitesse linéaire < 0.1
            """
        elif (STATE == "Atteint max"):
            #print(centers)
            if (Vr < 0.1):
                print("Valeur du potentiel final : ",potentialValue)
                print("Valeur du potentiel maximum :",pot.value([pot.mu1[0],pot.mu1[1]]))
                print("Erreur relative :",(pot.value([pot.mu1[0],pot.mu1[1]])-potentialValue)/pot.value([pot.mu1[0],pot.mu1[1]])*100,"%")
                STATE = "Fini"

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
print("Fin de la simulation")
print("==================================================")

# ========================================================================== #
#                   AFFICHAGE
# ========================================================================== #
# close all figures
plt.close("all")

# generate plots
fig,ax = simu.plotXY(1,-50,50,-50,50)
pot.plot(noFigure=None, fig=fig, ax=ax)  # plot potential for verification of solution

"""
Affichage de point intéressant pour la compréhension de la déterrmination du max.
"""
for point in outsidePoints:
    plt.plot(point[0], point[1], 'bo--', linewidth=2, markersize=2)
for point in potentialPoints:
    plt.plot(point[0], point[1], 'ro--', linewidth=2, markersize=2)
plt.plot(predictionPoint[0], predictionPoint[1], 'go--', linewidth=2, markersize=10)
for cluster in clusters:
    plt.plot(cluster["center"][0], cluster["center"][1], 'wo--', linewidth=2, markersize=5)
plt.plot(pot.mu1[0],pot.mu1[1], 'bo--', linewidth=2, markersize=3)
plt.plot(robot.x,robot.y, 'go--', linewidth=2, markersize=3)

# Animation *********************************
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
