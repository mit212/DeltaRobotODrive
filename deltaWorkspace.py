#!/usr/bin/python
"""
Delta Robot Main Interface

Daniel J. Gonzalez - dgonz@mit.edu
2.12 Intro to Robotics Spring 2019
"""
#####################################

import robot212_virtual as bot

import kinematicsSolver as kin
import time
import numpy as np

check_constraints = True
pi = np.pi #3.1415927
bot.trajMoveRad((0,0,0))
deltaKin = kin.deltaSolver()

if __name__ == "__main__":
    isRunning = True
    mainRunning = True
    inp = input("Press ENTER to begin or q+ENTER to quit...")
    if inp == 'q':
        mainRunning = False
    while mainRunning:
        tStart = time.time()
        while isRunning:
            t = time.time() - tStart
            #-400, 400 is the limits of xy plane on the plotter which you can see on kinematicsSolver.py
            #-900, 100 is the limits of the z axis on the plotter which you can see on kinematicsSolver.py
            xvec = np.linspace(-400, 400, 10)
            yvec = np.linspace(-400, 400, 10)
            zvec = np.linspace(-900, 100, 10)
            for i in range(10):
                for j in range(10):
                    for k in range(10):
                        # create grid
                        xD = xvec[i]
                        yD = yvec[j]
                        zD = zvec[k]
                        #   Solve for and execute trajectory, print the endpoint and joint angles, update the plot. 
                        thtDes = deltaKin.IK((xD, yD, zD))
                        xend = deltaKin.FK(thtDes)
                        if (abs(xend[0] - xD) > 0.1 or abs(xend[1] - yD) > 0.1 or abs(xend[2] - zD > 0.1)):
                            #Does not work so plot red
                            deltaKin.updatePlot((xD, yD, zD), 'red')
                        elif(check_constraints):
                            #check_constraints ONLY checks the physical constraint of the ball_swivel joint. 
                            #You can see the changes that will occur to the workspace by editting kinematicsSolver.py
                            motor1 = deltaKin.check_constraints(1, [xD, yD, zD], thtDes[0])
                            motor2 = deltaKin.check_constraints(2, [xD, yD, zD], thtDes[1])
                            motor3 = deltaKin.check_constraints(3, [xD, yD, zD], thtDes[2])
                            if(motor1 and motor2 and motor3):
                                deltaKin.updatePlot((xD, yD, zD), 'blue')
                            else:
                                deltaKin.updatePlot((xD, yD, zD), 'green')
                        else:
                            #plot blue because it does work
                            deltaKin.updatePlot((xD, yD, zD), 'blue')

                        print(str(i) + ',' + str(j) + ',' + str(k))
            isRunning = False 

        inp = input("Press ENTER to run again or q+ENTER to quit...")
        if inp == 'q':
            mainRunning = False
        else:
            isRunning = True
    bot.trajMoveRad((0,0,0))