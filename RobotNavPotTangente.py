"""
Projet Introduction à la robotique
FORDANT - GOUDIN - POIRIER

Méthode des tangentes
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
x0 = -20.0
y0 = -20.0
theta0 = 0.00001
robot = rob.Robot(x0, y0, theta0)

# potential
difficulty=1
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
tf = 50.0
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
STATE = "Initialisation"
tangentes = []

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
        Pendant l'initialisation, le robot se déplace en ligne droite jusqu'à trouver
        un potentiel de pollution valide (>0). Cette étape est nécessaire pour eviter
        de trouver une tangente dans une zone non valide.
        """
        # algo
        if (STATE == "Initialisation"):
            aim = move(2.0,0.0)
            if (potentialValue > 0):
                STATE = "Recherche"
                
            """
            Etape 2 :
            On cherche les deux tangentes. Si la valeur de potentiel augmente, on continu à avancer.
            Si la valeur est inférieur à la précédente (ΔP < 0), alors cela siginifie que le robot
            se trouve sur un point tangent et on récupère donc les données nécessaires : x,y,theta
            """   
        elif (STATE == "Recherche"):
            if(len(tangentes) < 2):
                if(potentialValues["variation"] < 0):
                    tangentes.append([robot.x,robot.y,robot.theta])
                    aim = move(1.0,0.2)
                else:
                    aim = move(2.0,0.0)
            else:
                STATE = "Calcul"
                
            """
            Etape 3 :
            On exécute les calculs afin d'obtenir le point d'intersection des perpendiculaires des tangentes.
            Puis on attribut ce point à l'objectif du robot.
            """     
        elif (STATE == "Calcul"):
            print("Les deux tangentes on été trouvé, les données du calcul sont les suivantes : ",tangentes)
            a1 = np.sin(tangentes[0][2]+np.pi/2)/np.cos(tangentes[0][2]+np.pi/2)
            a2 = np.sin(tangentes[1][2]+np.pi/2)/np.cos(tangentes[1][2]+np.pi/2)
            
            b1 = tangentes[0][1] - a1 * tangentes[0][0]
            b2 = tangentes[1][1] - a2 * tangentes[1][0]
            
            x = (b2 - b1) / (a1 - a2)
            y = a1 * x + b1
            aim = [x,y]
            print("Les coordonnées du maximum sont :",aim)
            STATE = "Conclusion"
         
            """
            Etape 4 :
            On attend ici l'atteinte du maximum trouvé lors de l'étape précédente.
            On considère qu'on l'a atteint quand le robot à une vitesse linéaire < 0.1
            """
        elif (STATE == "Conclusion"):
            if (Vr < 0.1):
                print("Le point maximum a été atteint, valeur du potentiel final :",potentialValue)
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
fig,ax = simu.plotXY(1)
pot.plot(noFigure=None, fig=fig, ax=ax)  # plot potential for verification of solution

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

