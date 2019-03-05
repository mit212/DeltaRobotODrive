#!/usr/bin/python
"""
Delta Robot Main Interface

Daniel J. Gonzalez - dgonz@mit.edu
2.12 Intro to Robotics Spring 2019
"""
#####################################
realBot = False
#####################################

if realBot:
    import robot212_odrive as bot
else:
    import robot212_virtual as bot

import kinematicsSolver as kin
import time
import numpy as np

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
            zD0 = deltaKin.z
            if t > 20:
                isRunning = False
            elif t<10:
                #   Define Trajectory
                r = (t/10)*100 #mm
                freq = 0.5#Hz
                xD = r*np.cos((freq*2*np.pi)*t)
                yD = r*np.sin((freq*2*np.pi)*t)
                zD = zD0 - (t/10)*300

                #   Solve for and execute trajectory, print the endpoint and joint angles, update the plot. 
                thtDes = deltaKin.IK((xD, yD, zD))
                bot.trajMoveRad(thtDes, 2*pi/8, 2*pi/8) # (Desired Angles [rad], Max Velocity [rad/s], Acceleration/Deceleration [rad/s^2])
                print((xD, yD, zD), thtDes)
                deltaKin.updatePlot((xD, yD, zD))
            elif t>10 and t<20:
                #   Define Trajectory
                r = 100 - ((t-10)/10)*100 #mm
                freq = 0.5#Hz
                xD = r*np.cos((freq*2*np.pi)*t)
                yD = r*np.sin((freq*2*np.pi)*t)
                zD = zD0 - 300 + ((t-10)/10)*300

                #   Solve for and execute trajectory, print the endpoint and joint angles, update the plot. 
                thtDes = deltaKin.IK((xD, yD, zD))
                bot.trajMoveRad(thtDes, 2*pi/8, 2*pi/8) # (Desired Angles [rad], Max Velocity [rad/s], Acceleration/Deceleration [rad/s^2])
                print((xD, yD, zD), thtDes)
                deltaKin.updatePlot((xD, yD, zD))
        inp = input("Press ENTER to run again or q+ENTER to quit...")
        if inp == 'q':
            mainRunning = False
        else:
            isRunning = True
    bot.trajMoveRad((0,0,0))