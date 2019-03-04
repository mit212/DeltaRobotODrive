#!/usr/bin/python
"""
Delta Robot Main Interface

Daniel J. Gonzalez - dgonz@mit.edu
2.12 Intro to Robotics Spring 2019
"""

import robot212_odrive as bot
import kinematicsSolver as kin
import time
import numpy as np

pi = np.pi #3.1415927
bot.trajMoveRad((0,0,0))
deltaKin = kin.deltaSolver()

if __name__ == "__main__":
    isRunning = True
    input("Press Enter to begin...")
    tStart = time.time()
    while isRunning:
        t = time.time() - tStart
        zD0 = deltaKin.z
        if t > 20:
            isRunning = False
        elif t<10:
            r = (t/10)*10 #mm
            freq = 0.5#Hz
            xD = r*np.cos((freq*2*np.pi)*t)
            yD = r*np.sin((freq*2*np.pi)*t)
            zD = zD0 - (t/10)*300
            thtDes = deltaKin.IK((xD, yD, zD))
            print((xD, yD, zD), thtDes)
            deltaKin.updatePlot((xD, yD, zD))
            bot.trajMoveRad(thtDes, 2*pi/8, 2*pi/8)
        elif t>10 and t<20:
            r = 10 - ((t-10)/10)*10 #mm
            freq = 0.5#Hz
            xD = r*np.cos((freq*2*np.pi)*t)
            yD = r*np.sin((freq*2*np.pi)*t)
            zD = zD0 - 300 + ((t-10)/10)*300
            thtDes = deltaKin.IK((xD, yD, zD))
            print((xD, yD, zD), thtDes)
            deltaKin.updatePlot((xD, yD, zD))
            bot.trajMoveRad(thtDes, 2*pi/8, 2*pi/8)
    bot.trajMoveRad((0,0,0))