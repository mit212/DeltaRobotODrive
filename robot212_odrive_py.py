from __future__ import print_function
"""
Created on Thu Jun 28 16:34:02 2018

@author: beeman, dgonz
"""
"""
Example usage of the ODrive python library to monitor and control ODrive devices
"""

import odrive
from odrive.enums import *
import time
import datetime
import math
import fibre
import numpy as np

import serial
import struct
import signal
import sys
import xrl_kinematics as xrlk
from xrl_kinematics import *
from dataLogger import *
import xrl_odrive as xrlo

in2mm = 25.4
mm2in = 1/in2mm
in2m = in2mm/1000

zeroVec = [[[0,0],[0,0],[0,0]],[[0,0], [0,0], [0,0]]]
home_thts = [[[0,0],[0,0],[0,0]],[[0,0], [0,0], [0,0]]]

#SAGITTAL = 0 | FRONTAL = 1
home_kp = [[[300, 300], [450, 450], [1000, 2000]], [[300, 300], [450, 450], [1000, 2000]]]
home_kd = [[[2.5,2.5], [3.7, 3.7], [5,5]], [[2.5,2.5], [3.7, 3.7], [5,5]]]

m_act = 8.73 #kg
m_batt = 0.9
m_payload = 0
m_torso = 2*m_act + 2*m_batt + m_payload
m_total = m_torso + 4*m_act
g = 9.80665 #m/s^2
F_0 = np.array([0, 0, m_total*g, 0, 0, 0]).T
F_0 = np.array([0, 0, 0, 0, 0, 0]).T
virtualDisconnect = False

thtDesired = [[[0,0],[0,0],[0,0]],[[0,0], [0,0], [0,0]]]
velDesired = [[[0,0],[0,0],[0,0]],[[0,0], [0,0], [0,0]]]
kPDesired = [[[0,0],[0,0],[0,0]],[[0,0], [0,0], [0,0]]]
kDDesired = [[[0,0],[0,0],[0,0]],[[0,0], [0,0], [0,0]]]

jointError = [[0, 0, 0],[0, 0, 0]]

gear_ratios = [32.0/15.0, 48.0/15.0, 36.0/15.0]

CPR2RAD = (2*math.pi/16384.0)   

thtActual = [[[0,0],[0,0],[0,0]],[[0,0], [0,0], [0,0]]]
velActual =   [[[0,0],[0,0],[0,0]],[[0,0], [0,0], [0,0]]]
curCommand = [[[0,0],[0,0],[0,0]],[[0,0], [0,0], [0,0]]]
ff = [[[0,0],[0,0],[0,0]],[[0,0], [0,0], [0,0]]]
p = FK(np.array(thtActual))

now = datetime.datetime.now()
myLoggerName = './trials/'+str(now.strftime("%Y-%m-%d__%H:%M")) + ".txt"
myLogger = dataLogger(myLoggerName)

seconds = 0
height = 0
max_height = 53.7241049455
wait_down = False
dynamic_gains = False
#BAUD = 921600

squat_home = xrlk.FrontalIK(0,max_height*in2m)
offsets = [[[-0.060416846055225015, -0.14392822980880737], [0.07375481578789378, -0.08451183140277863], [-0.0701616294209031, 0.00882530678063631]],\
         [[-0.03448943652553904, 0.0927095040678978], [0.12114781740389491, -0.06754493713378906], [-0.17290639127531326, 0.12108983099460602]]]

squat_offsets = offsets

pd = np.array([0, 0, max_height*in2m, 0, 0, 0])
K_p = [[500, 0, 0, 0, 0, 0],\
        [0, 500, 0, 0, 0, 0],\
        [0, 0, 0, 0, 0, 0],\
        [0, 0, 0, 100, 0, 0],\
        [0, 0, 0, 0, 0, 0],\
        [0, 0, 0, 0, 0, 100]]
B_p = [[5, 0, 0, 0, 0, 0],\
        [0, 5, 0, 0, 0, 0],\
        [0, 0, 0, 0, 0, 0],\
        [0, 0, 0, 5, 0, 0],\
        [0, 0, 0, 0, 0, 0],\
        [0, 0, 0, 0, 0, 5]]
S = [[0, 0, 0, 0, 0, 0],\
    [0, 0, 0, 0, 0, 0],\
    [0, 0, 1, 0, 0, 0],\
    [0, 0, 0, 0, 0, 0],\
    [0, 0, 0, 0, 1, 0],\
    [0, 0, 0, 0, 0, 0]]
SPerp = np.eye(6) - np.array(S)

print("squat home")
print(squat_home)
print("original offsets")
print(offsets)
'''
for i in range(0,len(offsets)):
    for j in range(0,len(offsets[0])):
        offsets[i][j][1] += squat_home[i][j]
print("adjusted offsets")
print(offsets)
'''

# [[right hip, right knee, right ankle], [left hip, left knee, left ankle]]
odrvs = [[None, None, None], [None, None, None]]
usb_serials = [['367333693037', '375F366E3137', '366933693037'], ['376136583137', '366E33683037', '366933683037']]
#usb_serials = [[None, None, None], [None, None, None]]

def connect_all():
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            # if not connecting to one of the odrives, will just pass over it
            # similar lines throughout program
            if usb_serials[leg][joint] == None:
                continue
            print("finding odrive: " + usb_serials[leg][joint] + "...")
            odrvs[leg][joint] = odrive.find_any(serial_number = usb_serials[leg][joint])
            print("found odrive! leg: " + str(leg) + ", joint: " + str(joint))
def connect_one(leg, joint):
    print("finding odrive: " + usb_serials[leg][joint] + "...")
    odrvs[leg][joint] = odrive.find_any(serial_number = usb_serials[leg][joint])
    print("found odrive! leg: " + str(leg) + ", joint: " + str(joint))

isRunning = True
t = 0

def printErrorStates():
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            print('leg',leg, ' joint',joint, ' axis0 error:',bin(odrvs[leg][joint].axis0.error))
            print('leg',leg, ' joint',joint, ' axis1 error:',bin(odrvs[leg][joint].axis1.error))
            print('leg',leg, ' joint',joint, ' motor0 error:',bin(odrvs[leg][joint].axis0.motor.error))
            print('leg',leg, ' joint',joint, ' motor1 error:',bin(odrvs[leg][joint].axis1.motor.error))
            print('leg',leg, ' joint',joint, ' encoder0 error:',bin(odrvs[leg][joint].axis0.encoder.error))
            print('leg',leg, ' joint',joint, ' encoder1 error:',bin(odrvs[leg][joint].axis1.encoder.error))
            print('leg',leg, ' joint',joint, ' axis0 debug:',odrvs[leg][joint].axis0.debug)
            print('leg',leg, ' joint',joint, ' axis1 debug:',odrvs[leg][joint].axis1.debug)

def main():
    global thtDesired, velDesired, kP, kD, thtActual, velActual, curCommand, t, offsets
    global kPDesired, kDDesired, pd, K_p, B_p, S, SPerp, home_kp, F_0
    global seconds, height, max_height, wait_down

    #connect to all odrives
    connect_all()
    xrlo.odrvs = odrvs

    ### LOGGER
    #myLogger.appendData('\n--NewTrial--\n') #removing this makes it easier to import into MATLAB
    titleStr = 't,'
    # for i in range(0,len(odrvs)):
    #     for j in range(0,len(odrvs[0])):
    #         for k in range(0,2):
    #             titleStr+='thtDesired'+str(i)+str(j)+str(k)+','
    # for i in range(0,len(odrvs)):
    #     for j in range(0,len(odrvs[0])):
    #         for k in range(0,2):
    #             titleStr+='thtActual'+str(i)+str(j)+str(k)+','
    # for i in range(0,len(odrvs)):
    #     for j in range(0,len(odrvs[0])):
    #         for k in range(0,2):
    #             titleStr+='cmd'+str(i)+str(j)+str(k)+','
    titleStr+='x,y,z'
    myLogger.appendData(titleStr)

    ###INITIAL STATE
    state = 'home'

    ###SET CONTROL MODES
    xrlo.mixed_config_all()
    xrlo.closed_loop_state_all()

    ### SET THE MIXED GAINS
    ## 2x CHANGE MADE BY DGONZ:
    #xrlo.ramp_up_gains_all_sagittal(0, 0.0) #use cpr2rad if NOT in mixed mode
    #xrlo.ramp_up_gains_all_frontal(0 ,0.0)
    xrlo.set_gear_ratios()

    isRunning = True
    t = 0
    tStart = time.time()
    commAll()
    print('-----------------Begin')
    print('-------------------------------State: ',state)
    while(isRunning):
        time.sleep(0.010)
        t = time.time() - tStart
        if(state == 'home'): # homing sequence
            # send desired home pose
            rampTime = 5
            #thtDesired = zeroVec
            velDesired = zeroVec
            #set thtDesired to standing position to start
            #thtVals = [[0,0,0],[0,0,0]]
            #thtVals = xrlk.FrontalIK(0,53.7*in2m) #changed to rads
            for i in range(0,len(odrvs)):
                for j in range(0,len(odrvs[0])):
                    thtDesired[i][j][0] = squat_home[i][j]+offsets[i][j][0]
                    thtDesired[i][j][1] = 0+offsets[i][j][1] #+ thtVals[i][j]
            #commAll()

            # ramp up kD and kP to high for some time
            #xrlo.max_gains_all()
            print("Home")
            # if near home pose or if ramptime is complete, change to idle state.
            if(t>=rampTime):
                #state = 'squatdown'
                #state = 'waitforsquat'
                #state = 'holdZero'
                state = 'configure'
                print('-------------------------------State: ',state)
                tStartSquat = t
        elif(state == 'holdZero'):
            state = 'holdZero'
            myLogger.appendData(str([t,thtDesired,thtActual,curCommand]))
        elif(state =='configure'):
            while True:
                i = input("Press q+Enter to quit or one of the following to configure:\ntorque constant\npll\ngear ratio\
                \nprint errors\nprint gains\nprint pos\nprint all\nthts\ngains\ninits\ntests\ndynamic gains\nsquat\nhocca...")
                if i=='q':
                    cleanQuit()
                elif i=='hocca':
                    state = 'waitforhocca'
                    break
                elif i=='squat':
                    state = 'waitforsquat'
                    break
                elif i=='tests':
                    state = 'tests'
                    break
                elif i=='inits':
                    state = 'inits'
                    break
                elif i=='gains':
                    state = 'gains'
                    break
                elif i=='thts':
                    state = 'thts'
                    break
                elif i=='dynamic gains tests':
                    dynamic_gains = get_bool_from_user('activate dynamic gains')
                    state = 'dynamic gains'
                    break

                ###configuartion parameters
                elif i=='gear ratio':
                    hip_ratio = get_float_num_from_user('hip gear ratio', 0, 100)
                    knee_ratio = get_float_num_from_user('knee gear ratio', 0, 100)
                    ankle_ratio = get_float_num_from_user('ankle gear ratio', 0, 100)
                    xrlo.gear_ratios_all(hip_ratio, knee_ratio, ankle_ratio)
                elif i=='torque constant':
                    leg = get_int_num_from_user('leg', range(2))
                    joint = get_int_num_from_user('joint', range(3))
                    motor = get_int_num_from_user('motor', range(2))
                    torque_constant = get_float_num_from_user('torque_constant', 0, 100)
                    xrlo.set_torque_constant(torque_constant, leg, joint, motor)
                elif i=='pll':
                    leg = get_int_num_from_user('leg', range(2))
                    joint = get_int_num_from_user('joint', range(3))
                    pll_bandwidth = get_int_num_from_user('pll', range(10000))
                    xrlo.set_pll(leg, joint, pll_bandwidth)

                ###print commands
                elif i=='print all':
                    print(odrvs)
                elif i=='print errors':
                    xrlo.printErrorStates()
                elif i=='print gearing':
                    xrlo.print_controller()
                elif i=='print gains':
                    print("Sagittal kp: " + str(xrlo.get_sagittal_kp_gains_all()))
                    print("Sagittal kd: " + str(xrlo.get_sagittal_kd_gains_all()))
                    print("Frontal kp: " + str(xrlo.get_frontal_kp_gains_all()))
                    print("Frontal kd: " + str(xrlo.get_frontal_kd_gains_all()))
                elif i=='print pos':
                    print("Positions: " + str(xrlo.get_pos_all()))

        elif(state =='thts'):
            while True:
                i = input("Press Enter to return, q+Enter to quit, or one of the following to configure:\ngo home\nread thts minus offsets\ndes thts minus offsets\nread thts\ndes thts\noffsets\nsend thts\nset home...")
                if not i:
                    state = 'configure'
                    break
                if i=='q':
                    cleanQuit()
                elif i=='read thts minus offsets':
                    pos_minus_offsets = xrlo.read_thts()
                    for i in range(0,len(odrvs)):
                        for j in range(0,len(odrvs[0])):
                            pos_minus_offsets[i][j][0] = pos_minus_offsets[i][j][0] - offsets[i][j][0]
                            pos_minus_offsets[i][j][1] = pos_minus_offsets[i][j][1] - offsets[i][j][1]
                            #htDesired[i][j][1] = 0+offsets[i][j][1] #+ thtVals[i][j
                    print("Positions minus offsets: " + str(pos_minus_offsets))
                elif i=='des thts minus offsets':
                    pos_minus_offsets = xrlo.read_thts()
                    for i in range(0,len(odrvs)):
                        for j in range(0,len(odrvs[0])):
                            pos_minus_offsets[i][j][0] = thtDesired[i][j][0] - offsets[i][j][0]
                            pos_minus_offsets[i][j][1] = thtDesired[i][j][1] - offsets[i][j][1]
                            #htDesired[i][j][1] = 0+offsets[i][j][1] #+ thtVals[i][j
                    print("Desired position with offsets: " + str(pos_minus_offsets))
                elif i=='des thts':
                    print(thtDesired)
                elif i=='read thts':
                    print(xrlo.read_thts())
                elif i=='offsets':
                    print(offsets)
                elif i=='go home':
                    for i in range(0,len(odrvs)):
                        for j in range(0,len(odrvs[0])):
                            thtDesired[i][j][0] = squat_home[i][j]+offsets[i][j][0]
                            thtDesired[i][j][1] = 0+offsets[i][j][1] #+ thtVals[i][j]
                    commAll()
                elif i=='send thts':
                    for i in range(0,len(odrvs)):
                        for j in range(0,len(odrvs[0])):
                            thtDesired[i][j][0] = squat_home[i][j]+offsets[i][j][0]
                            thtDesired[i][j][1] = 0+offsets[i][j][1] #+ thtVals[i][j]
                    commAll()
                elif i=='set home':
                    home_thts = xrlo.read_thts()
                    for i in range(0,len(odrvs)):
                        for j in range(0,len(odrvs[0])):
                            home_thts[i][j][0] -= squat_home[i][j]
                    offsets = home_thts
                    print(offsets)
                    print("To permanently save this home position, copy the above array and paste into xrl_odrive_py as offsets.")
                    for i in range(0,len(odrvs)):
                        for j in range(0,len(odrvs[0])):
                            thtDesired[i][j][0] = squat_home[i][j]+offsets[i][j][0]
                            thtDesired[i][j][1] = 0+offsets[i][j][1] #+ thtVals[i][j]
                    commAll()

        elif(state =='gains'):
            while True:
                i = input("Press Enter to return, q+Enter to quit, or one of the following to configure:\nprint\nall\nmax all\nsingle joint\njoint pair...")
                if not i:
                    state = 'configure'
                    break
                if i=='q':
                    cleanQuit()
                elif i=='print':
                    print("Sagittal kp: " + str(xrlo.get_sagittal_kp_gains_all()))
                    print("Sagittal kd: " + str(xrlo.get_sagittal_kd_gains_all()))
                    print("Frontal kp: " + str(xrlo.get_frontal_kp_gains_all()))
                    print("Frontal kd: " + str(xrlo.get_frontal_kd_gains_all()))
                elif i=='max all':
                    kPDesired = home_kp
                    kDDesired = home_kd
                    xrlo.max_gains_all()
                elif i=='single joint':
                    #get parameters
                    leg = get_int_num_from_user('leg', range(2))
                    joint = get_int_num_from_user('joint', range(3))
                    #gear = get_bool_from_user('geared')
                    s_kp = get_float_num_from_user('sagittal kp', 0.0, 5000.0)
                    s_kd = get_float_num_from_user('sagittal kd', 0.0, 20.0)
                    f_kp = get_float_num_from_user('frontal kp', 0.0, 5000.0)
                    f_kd = get_float_num_from_user('frontal kd', 0.0, 20.0)
                    xrlo.ramp_up_gains(leg, joint, s_kp, s_kd, f_kp, f_kd, rampSec=5, hz=100, debug=False)
                elif i=='joint pair':
                    #get parameters
                    joint = get_int_num_from_user('joint', range(3))
                    #gear = get_bool_from_user('geared')
                    s_kp = get_float_num_from_user('sagittal kp', 0.0, 5000.0)
                    s_kd = get_float_num_from_user('sagittal kd', 0.0, 20.0)
                    f_kp = get_float_num_from_user('frontal kp', 0.0, 5000.0)
                    f_kd = get_float_num_from_user('frontal kd', 0.0, 20.0)
                    xrlo.ramp_up_gains(0, joint, s_kp, s_kd, f_kp, f_kd, rampSec=5, hz=100, debug=False)
                    xrlo.ramp_up_gains(1, joint, s_kp, s_kd, f_kp, f_kd, rampSec=5, hz=100, debug=False)
                elif i=='all':
                    #gear = get_bool_from_user('geared')
                    #get parameters
                    s_kp = get_float_num_from_user('sagittal kp', 0.0, 5000.0)
                    s_kd = get_float_num_from_user('sagittal kd', 0.0, 20.0)
                    f_kp = get_float_num_from_user('frontal kp', 0.0, 5000.0)
                    f_kd = get_float_num_from_user('frontal kd', 0.0, 20.0)

                    xrlo.ramp_up_gains_all_sagittal(s_kp, s_kd, rampSec=5, hz=100, debug=False)
                    xrlo.ramp_up_gains_all_frontal(f_kp,f_kd, rampSec=5, hz=100, debug=False)
        elif(state =='tests'):
            while True:
                i = input("Press Enter to return, q+Enter to quit, or one of the following to configure:\nposition\nposition all\nramptest\nramptest all...")
                if not i:
                    state = 'configure'
                    break
                if i=='q':
                    cleanQuit()
                elif i=='position':
                    leg = get_int_num_from_user('leg', range(2))
                    joint = get_int_num_from_user('joint', range(3))
                    seconds = get_int_num_from_user('seconds', range(1000))
                    rads = get_float_num_from_user('rads', -100, 100)
                    xrlo.spin_to_pos(leg, joint, seconds, rads)
                elif i=='position all':
                    seconds = get_int_num_from_user('seconds', range(1000))
                    rads = get_float_num_from_user('rads', -100, 100)
                    xrlo.spin_to_pos_all(seconds, rads)
                elif i=='ramptest':
                    leg = get_int_num_from_user('leg', range(2))
                    joint = get_int_num_from_user('joint', range(3))
                    seconds = get_int_num_from_user('seconds', range(1000))
                    rads = get_float_num_from_user('rads', -100, 100)
                    xrlo.ramp_test(leg, joint, seconds, rads)
                elif i=='ramptest all':
                    seconds = get_int_num_from_user('seconds', range(1000))
                    rads = get_float_num_from_user('rads', -100, 100)
                    xrlo.ramp_test_all(seconds, rads)
        elif(state =='inits'):
            while True:
                i = input("Press Enter to return, q+Enter to quit, or one of the following to configure:\ninit all\ninit joint\ninit motor\nmake perm\nmake perm all...")
                if not i:
                    state = 'configure'
                    break
                if i=='q':
                    cleanQuit()
                elif i=='init all':
                    reset = get_bool_from_user('reset')
                    xrlo.full_init(reset=reset)
                elif i=='init joint':
                    leg = get_int_num_from_user('leg', range(2))
                    joint = get_int_num_from_user('joint', range(3))
                    reset = get_bool_from_user('reset')
                    xrlo.joint_init(leg, joint, reset=reset)
                elif i=='init motor':
                    leg = get_int_num_from_user('leg', range(2))
                    joint = get_int_num_from_user('joint', range(3))
                    motor = get_int_num_from_user('motor', range(2))
                    reset = get_bool_from_user('reset')
                    print("leg: " + str(leg) + ", joint: " + str(joint) + ", motor: " + str(motor))
                    xrlo.single_init(leg,joint,motor, reset=reset)
                elif i=='make perm':
                    leg = get_int_num_from_user('leg', range(2))
                    joint = get_int_num_from_user('joint', range(3))
                    xrlo.make_perm(leg, joint)
                elif i=='make perm all':
                    xrlo.make_perm_all()
        elif(state == 'dynamic gains'):
            print("NOT IMPLEMENTED")
            print("RETURNING TO CONFIGURE")
            state = 'configure'

            while True:
                i = input("Press Enter to go back, exit, q+Enter to quit, or select:\nsampleTest...")
                if not i:
                    dynamic_gains = get_bool_from_user('keep dynamic gains activated?')
                    state = 'configure'
                    break
                elif i=='exit':
                    state = 'home'
                    break
                elif i=='q':
                    cleanQuit()
                elif i=='sampleTest':
                    seconds = get_float_num_from_user('seconds', 0, 100000)
                    tStartSquat = t-tStart
                    state = 'sampleDynGainsTest'

        elif(state == 'sampleDynGainsTest'):
            rampTime = seconds
            #need some sort of input for the xrlk functions, like squat has yDes
            kPVals = home_kp #xrlk.[unwritten dynamic gains function]
            kDVals = home_kd #xrlk.[unwritten dynamic gains function]
            update_des_gains(kPVals, kDVals)
            #if reach low position, stand back up
            if(t - tStartSquat >= rampTime):
                tStartUp = t
                state = 'sampleDynGainsTestReverse'
                print('-------------------------------State: ',state)

        elif(state == 'sampleDynGainsTestReverse'): # stand up over period of time
            #IK, log all feedback over time
            while wait_down:
                print("waiting")
                i = input("Press Enter to continue, or q+Enter to quit...")
                if not i:
                    wait_down = False
                    tStartUp = time.time() - tStart
                    t = time.time() - tStart
                    print("done waiting!")
                if i=='q':
                    cleanQuit()

            rampTime = seconds
            xDes = 0
            #For real robot: No lower than 32.75, midpoint is 42, max is 53.70
            if(t-tStartUp < rampTime):
                rampTime = seconds
                #need some sort of input for the xrlk functions, like squat has yDes
                kPVals = home_kp #xrlk.JacFront(np.array(flattenList(thtDesired)) #xrlk.[unwritten dynamic gains function]
                kDVals = home_kd #xrlk.[unwritten dynamic gains function]
                update_des_gains(kPVals, kDVals)
            if(t - tStartUp>= rampTime):
                kPVals = home_kp #this should always be home
                kDVals = home_kd
                update_des_gains(kPVals, kDVals)
                myLogger.writeOut()
                state = 'dynamic gains'
                print('-------------------------------State: ',state)

        elif(state == 'comboTest'):
            rampTime = seconds
            #need some sort of input for the xrlk functions, like squat has yDes
            kPVals = home_kp #xrlk.[unwritten dynamic gains function]
            kDVals = home_kd #xrlk.[unwritten dynamic gains function]
            yDes = (max_height-(((t - tStartSquat)/rampTime))*(max_height - height))*in2m
            thtVals = xrlk.FrontalIK(xDes,yDes)
            update_des_gains_and_pos(kPVals, kDVals, thtVals)
            #if reach low position, stand back up
            if(t - tStartSquat >= rampTime):
                tStartUp = t
                state = 'comboTestReverse'
                print('-------------------------------State: ',state)

        elif(state == 'comboTestReverse'): # stand up over period of time
            #IK, log all feedback over time
            while wait_down:
                print("waiting")
                i = input("Press Enter to continue, or q+Enter to quit...")
                if not i:
                    wait_down = False
                    tStartUp = time.time() - tStart
                    t = time.time() - tStart
                    print("done waiting!")
                if i=='q':
                    cleanQuit()

            rampTime = seconds
            #need some sort of input for the xrlk functions, like squat has yDes
            kPVals = home_kp #xrlk.[unwritten dynamic gains function]
            kDVals = home_kd #xrlk.[unwritten dynamic gains function]
            yDes = (max_height-(((t - tStartSquat)/rampTime))*(max_height - height))*in2m
            thtVals = xrlk.FrontalIK(xDes,yDes)

            update_des_gains_and_pos(kPVals, kDVals, thtVals)

            if(t - tStartUp>= rampTime):
                kPVals = home_kp #this should always be home
                kDVals = home_kd
                yDes = (max_height-(((t - tStartSquat)/rampTime))*(max_height - height))*in2m
                thtVals = xrlk.FrontalIK(xDes,yDes)

                update_des_gains_and_pos(kPVals, kDVals, thtVals)

                myLogger.writeOut()
                #state = 'idle'
                state = 'dynamic gains'
                #state = 'squatdown'
                #tStartSquat = t
                print('-------------------------------State: ',state)
        elif(state == 'waitforhocca'):
            for i in range(0,len(odrvs)):
                for j in range(0,len(odrvs[0])):
                    thtDesired[i][j][0] = squat_home[i][j]+offsets[i][j][0]
                    thtDesired[i][j][1] = 0+offsets[i][j][1] #+ thtVals[i][j]
            while True:
                i = input("Hybrid Open-Loop Closed-Loop Conrol Architecture (HOCCA)\nPress Enter to continue, or exit, configure, or q+Enter to quit...")
                if not i:
                    break
                if i=='configure':
                    state = 'configure'
                    break
                if i=='exit':
                    state = 'home'
                    break
                if i=='q':
                    cleanQuit()
            if state != 'configure':
                seconds = get_int_num_from_user('Seconds until virtual disconnect', range(1000))
                seconds2 = get_int_num_from_user('Seconds after disconnect until end', range(1000))
                print("Switching to Hybrid Open-Loop Closed-Loop Control Architecture\nin 3...")
                time.sleep(1)
                print("             2...")
                time.sleep(1)
                print("             1...")
                time.sleep(1)
                state = 'hocca'
                t = time.time() - tStart
                tStartHocca = t
        elif(state == 'hocca'):
            runTime = seconds
            xDes = 0
            if(t-tStartHocca < rampTime):

                pref = pd.T + SPerp @ p.T
                thtDesired = IK(pref)
                J = JacSquat(thtActual)

                K0 = np.diag(fl(home_kp)[0]) #Need this [0] here!
                B0 = np.diag(fl(home_kd)[0])

                Kq = Np(J) @ K0 + J.T @ K_p @ J # + Hessian bits
                Bq = Np(J) @ B0 + J.T @ B_p @ J

                ff = J.T @ F_0 #\
                #    + nondiag(Kq) @ np.subtract(fl(thtDesired).T, fl(thtActual).T) \
                #    + nondiag(Bq) @ np.subtract(fl(velDesired).T, fl(velActual).T)
                
                for i in range(0,len(odrvs)):
                    for j in range(0,len(odrvs[0])):
                        thtDesired[i][j][0] = thtDesired[i][j][0] + offsets[i][j][0]
                        thtDesired[i][j][1] = thtDesired[i][j][1] + offsets[i][j][1]
                
                kPDesired = home_kp #unfl(np.diag(Kq))
                kDDesired = home_kd #unfl(np.diag(Bq))
                ff = unfl(ff)
                print(pref)
                print(Kq, kPDesired)
                print(np.linalg.eigvals(Kq))
                print(Bq, kDDesired)
                print(ff)
            #After a certain amount of time, virtually disconnect
            elif(t - tStartHocca>= runTime):
                tStartDisconnect = t
                print(Kq, kPDesired)
                print(Bq, kDDesired)
                print(ff)
                state = 'virtualdisconnect'
                print('-------------------------------State: ',state)

        elif(state == 'virtualdisconnect'):
            runTime2 = seconds2
            virtualDisconnect = True
            if(t - tStartDisconnect < runTime2):
                pass
            #If done, stand back up
            if(t - tStartDisconnect >= runTime2):
                myLogger.writeOut()
                state = 'waitforhocca'
                print('-------------------------------State: ',state)
        elif(state == 'waitforsquat'):
            for i in range(0,len(odrvs)):
                for j in range(0,len(odrvs[0])):
                    thtDesired[i][j][0] = squat_home[i][j]+offsets[i][j][0]
                    thtDesired[i][j][1] = 0+offsets[i][j][1] #+ thtVals[i][j]
            while True:
                i = input("Press Enter to continue, or exit, configure, or q+Enter to quit...")
                if not i:
                    break
                if i=='configure':
                    state = 'configure'
                    break
                if i=='exit':
                    state = 'home'
                    break
                if i=='q':
                    cleanQuit()
            if state != 'configure':
                wait_down = get_bool_from_user('wait at bottom')
                seconds = get_int_num_from_user('seconds', range(1000))
                height = get_float_num_from_user('height to squat to (max 53.7, min 33) in inches', 32.75, max_height)
                print("Squatting in 3...")
                time.sleep(1)
                print("             2...")
                time.sleep(1)
                print("             1...")
                time.sleep(1)
                state = 'squatdown'
                print('-------------------------------State: ',state)
                t = time.time() - tStart
                tStartSquat = t

        elif(state == 'idle'): # wait for user command to perform a squat
            # if user hits enter?
            state = 'idle'
            # if user hits q
            #state = 'quit'
        elif(state == 'squatdown' ): # squat down over period of time
            rampTime = seconds
            xDes = 0
            #For real robot: No lower than 32.75, midpoint is 42, max is 53.70
            if(t-tStartSquat < rampTime):
                yDes = (max_height-(((t - tStartSquat)/rampTime))*(max_height - height))*in2m
                thtVals = xrlk.FrontalIK(xDes,yDes)
                update_des_pos(thtVals)
            #if reach low position, stand back up
            if(t - tStartSquat>= rampTime):
                tStartUp = t
                state = 'standup'
                print('-------------------------------State: ',state)
        elif(state == 'standup'): # stand up over period of time
            #IK, log all feedback over time
            while wait_down:
                print("waiting")
                i = input("Press Enter to continue, or q+Enter to quit...")
                if not i:
                    wait_down = False
                    tStartUp = time.time() - tStart
                    t = time.time() - tStart
                    print("done waiting!")
                if i=='q':
                    cleanQuit()

            rampTime = seconds
            xDes = 0
            #For real robot: No lower than 32.75, midpoint is 42, max is 53.70
            if(t-tStartUp < rampTime):
                yDes = (max_height-(1-((t - tStartUp)/rampTime))*(max_height - height))*in2m
                #print(yDes)
                thtVals = xrlk.FrontalIK(xDes,yDes) #because of current robot orientation, FrontalIK returns values that are supplied to SAGITTAL motors
                #print("theta vals")
                #print(thtVals)
                update_des_pos(thtVals)

            #If reach top, idle
            if(t - tStartUp>= rampTime):
                update_des_pos(thtVals)
                myLogger.writeOut()
                state = 'waitforsquat'
                print('-------------------------------State: ',state)
        elif(state == 'quit'): # quit
            cleanQuit()
        commAll(state, t)
def commAll(state = '', t = 0):
    global thtDesired, velDesired, kP, kD, thtActual, velActual, curCommand, offsets
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            if odrvs[leg][joint] == None:
                print('NOT DETECTED')
                continue
            odrv_comm(leg, joint)
    #print('Des:',niceList(thtDesired))
    #print('Act:',niceList(thtActual))

def cleanQuit():
    global kP, kD, t, myLogger
    myLogger.writeOut()
    print("\n-----------------Interrupt received")
    print('Time of End: ',t)
    isRunning = False
    print('-----------------Quitting...')
    print('-----------------Ramping Gains Down...')
    #deal with gains
    dynamic_gains = False
    xrlo.ramp_up_gains_all_sagittal(0, 0.0, rampSec=1) #use cpr2rad if NOT in mixed mode
    xrlo.ramp_up_gains_all_frontal(0 ,0.0, rampSec=1)
    print('-----------------All Gains at low')

    #deal with odrive axis states
    for leg in range(0,len(odrvs)):
        for joint in range(0,len(odrvs[0])):
            if odrvs[leg][joint] == None:
                continue
            odrvs[leg][joint].axis0.requested_state = AXIS_STATE_IDLE
            odrvs[leg][joint].axis1.requested_state = AXIS_STATE_IDLE
    print('-----------------All Motors idle')

    '''
    #thtVals, thtDesired always get reset whenever we boot up
    #so this isn't necessary
    thtVals = xrlk.FrontalIK(0,53.7*in2m)
    for i in range(0,len(odrvs)):
        for j in range(0,len(odrvs[0])):
                    thtDesired[i][j][0] = -thtVals[i][j]+offsets[i][j][0]
                    thtDesired[i][j][1] = 0+offsets[i][j][1] #+ thtVals[i][j]
    '''
    commAll()
    print('-----------------Quit!')
    sys.exit(0)

def odrv_comm(leg, joint):
    global thtDesired, velDesired, kPDesired, kDDesired, ff, thtActual, velActual, curCommand, offsets, p
    ### Send Commands

    ###Mixed Position Control
    #both, sagittal, frontal
    #odrvs[leg][joint].axis0.controller.set_mixed_pos_setpoint(True, thtDesired[leg][joint][0], 0)

    ###Mixed Pos&Vel Control
    #both, sagittal, frontal, sagittal_vel, frontal_vel
    #odrvs[leg][joint].axis0.controller.set_mixed_setpoint(True, thtDesired[leg][joint][0], thtDesired[leg][joint][1], 0, 0) 
    #velDesired[leg][joint][0] as second to last term
    #last argument should eventually be velDesired[leg][joint][1]

    ###Full Jointspace PD Pos control with gain updates and feedforward torques.
    thtS = thtDesired[leg][joint][0] 
    thtF = thtDesired[leg][joint][1]
    kpS = kPDesired[leg][joint][0]
    kpF = kPDesired[leg][joint][1]
    kdS = kDDesired[leg][joint][0]
    kdF = kDDesired[leg][joint][1]
    ffS = ff[leg][joint][0]
    ffF = ff[leg][joint][1]
    if(not virtualDisconnect):
        odrvs[leg][joint].axis0.controller.set_mixed_pos_all(True, thtS, thtF, kpS, kpF, kdS, kdF, ffS, ffF)

    ### Read Current States
    #position
    thtActual[leg][joint][0] = odrvs[leg][joint].axis0.controller.theta_s - offsets[leg][joint][0]
    thtActual[leg][joint][1] = odrvs[leg][joint].axis0.controller.theta_f - offsets[leg][joint][1]
    #velocity
    velActual[leg][joint][0] = odrvs[leg][joint].axis0.controller.theta_s_dot
    velActual[leg][joint][1] = odrvs[leg][joint].axis0.controller.theta_f_dot
    #current
    #curCommand[leg][joint][0] = odrvs[leg][joint].axis0.motor.current_control.Iq_measured
    #curCommand[leg][joint][1] = odrvs[leg][joint].axis1.motor.current_control.Iq_measured
    (x,y,z,phi,theta,psi) = FK(np.array(thtActual))
    p = np.array((x,y,z,phi,theta,psi))
    myLogger.appendData(str([t,x,y,z]))

def cleanQuitInt(signal, frame):
    cleanQuit()
def niceList(theList):
    return ["%07.2f" % i for i in flattenList(theList)]

def flattenList(container):
    for i in container:
        if isinstance(i, (list,tuple)):
            for j in flattenList(i):
                yield j
        else:
            yield i

def full_init(reset = False):
    if(reset):
        for leg in range(len(odrvs)):
            for joint in range(len(odrvs[0])):
                odrvs[leg][joint].axis0.motor.config.pre_calibrated = False
                odrvs[leg][joint].axis0.encoder.config.pre_calibrated = False
                odrvs[leg][joint].axis1.motor.config.pre_calibrated = False
                odrvs[leg][joint].axis1.encoder.config.pre_calibrated = False
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):

            #motor current limit
            odrvs[leg][joint].axis0.motor.config.current_lim = 50
            odrvs[leg][joint].axis1.motor.config.current_lim = 50

            #Calibration Max Voltage
            odrvs[leg][joint].axis0.motor.config.resistance_calib_max_voltage = 4
            odrvs[leg][joint].axis1.motor.config.resistance_calib_max_voltage = 4

            #motor calibration current
            odrvs[leg][joint].axis0.motor.config.calibration_current = 15
            odrvs[leg][joint].axis1.motor.config.calibration_current = 15

            #brake resistance
            odrvs[leg][joint].config.brake_resistance = 0

            #axis state
            if(odrvs[leg][joint].axis0.motor.config.pre_calibrated == False):
                odrvs[leg][joint].axis0.encoder.set_pll_bandwidth(1570)
                time.sleep(1)
                odrvs[leg][joint].axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            if(odrvs[leg][joint].axis1.motor.config.pre_calibrated == False):
                # Change velocity pll bandwidth high temporarily to ensure calibration works well
                odrvs[leg][joint].axis1.encoder.set_pll_bandwidth(1570)
                time.sleep(1)
                odrvs[leg][joint].axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
                time.sleep(1)
    print("Done doing setup.")
    time.sleep(20)
    print("Saving Configuration...")
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            # Change velocity pll bandwidth back
            odrvs[leg][joint].axis0.encoder.set_pll_bandwidth(314)
            odrvs[leg][joint].axis1.encoder.set_pll_bandwidth(314)

            #motor and encoder pre_calibrated
            if(odrvs[leg][joint].axis0.error == 0):
                odrvs[leg][joint].axis0.motor.config.pre_calibrated = True
                odrvs[leg][joint].axis0.encoder.config.pre_calibrated = True
            else:
                odrvs[leg][joint].axis0.motor.config.pre_calibrated = False
                odrvs[leg][joint].axis0.encoder.config.pre_calibrated = False

            if(odrvs[leg][joint].axis1.error == 0):
                odrvs[leg][joint].axis1.motor.config.pre_calibrated = True
                odrvs[leg][joint].axis1.encoder.config.pre_calibrated = True
            else:
                odrvs[leg][joint].axis1.motor.config.pre_calibrated = False
                odrvs[leg][joint].axis1.encoder.config.pre_calibrated = False

            #gear ratio
            odrvs[leg][joint].axis0.controller.config.gear_ratio = 1#gear_ratios[joint]
            odrvs[leg][joint].axis1.controller.config.gear_ratio = 1#gear_ratios[joint]

            #gear ratio
            odrvs[leg][joint].axis0.controller.config.torque_constant = 0.45#Nm/A
            odrvs[leg][joint].axis1.controller.config.torque_constant = 0.45

            #Control Mode
            odrvs[leg][joint].axis0.controller.config.control_mode = 4 #Switch to pure Impedance control
            odrvs[leg][joint].axis1.controller.config.control_mode = 4

            #Set closed loop gains
            odrvs[leg][joint].axis0.controller.config.pos_gain = 0.0005
            odrvs[leg][joint].axis1.controller.config.pos_gain = 0.0005
            odrvs[leg][joint].axis0.controller.config.vel_gain = 0.00005
            odrvs[leg][joint].axis1.controller.config.vel_gain = 0.00005

            #axis state
            odrvs[leg][joint].axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            odrvs[leg][joint].axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

            # save configuration
            odrvs[leg][joint].save_configuration()

    print("Done initializing!")



###sets kP and kD to kPVals and kDVals, sets thtDesired to thtSagVals and thtFrontVals, accounts for offsets, logs
def update_des_gains_and_pos(kPVals, kDVals, thtSagVals, thtFrontVals=0):
    global thtDesired, velDesired, kP, kD, thtActual, velActual, curCommand, t, offsets
    global seconds, height, max_height, wait_down
    for i in range(0,len(odrvs)):
        for j in range(0,len(odrvs[0])):
            #sagittal kp
            kPDesired[i][j][0] = kPVals[leg][joint][0]
            #sagittal kd
            kDDesired[i][j][0] = kDVals[leg][joint][0]
            #frontal kp
            kPDesired[i][j][1] = kPVals[leg][joint][1]
            #frontal kd
            kDDesired[i][j][1] = kDVals[leg][joint][1]
            #set the position
            #sagittal tht
            thtDesired[i][j][0] = thtSagVals[i][j]+offsets[i][j][0]
            #frontal tht
            thtDesired[i][j][1] = 0+offsets[i][j][1] #+ thtFrontVals[i][j]
    myLogger.appendData(str([t,thtDesired,thtActual,curCommand]))

###sets thtDesired to thtSagVals, thtFrontVals, accounting for offsets, logs
def update_des_pos(thtSagVals, thtFrontVals=0):
    global thtDesired, velDesired, kP, kD, thtActual, velActual, curCommand, t, offsets
    global seconds, height, max_height, wait_down
    for i in range(0,len(odrvs)):
        for j in range(0,len(odrvs[0])):
            #set the position
            #sagittal tht
            thtDesired[i][j][0] = thtSagVals[i][j]+offsets[i][j][0]
            #frontal tht
            thtDesired[i][j][1] = 0+offsets[i][j][1] #+ thtFrontVals[i][j]
    myLogger.appendData(str([t,thtDesired,thtActual,curCommand]))

###sets kP and kD to kPVals and kDVals, accounts for offsets, logs
def update_des_gains(kPVals, kDVals):
    global thtDesired, velDesired, kP, kD, thtActual, velActual, curCommand, t, offsets
    global seconds, height, max_height, wait_down
    for i in range(0,len(odrvs)):
        for j in range(0,len(odrvs[0])):
            #sagittal kp
            kPDesired[i][j][0] = kPVals[leg][joint][0]
            #sagittal kd
            kDDesired[i][j][0] = kDVals[leg][joint][0]
            #frontal kp
            kPDesired[i][j][1] = kPVals[leg][joint][1]
            #frontal kd
            kDDesired[i][j][1] = kDVals[leg][joint][1]
    myLogger.appendData(str([t,thtDesired,thtActual,curCommand]))



def get_int_num_from_user(item_str, allowed_range):
    #use keyboard input to get a number from within an allowed range from the user
    result = None
    while True:
        #ask for input
        i = input("Press q+Enter to quit, or enter " + item_str + "...")
        #exit the function and cleanQuit
        if i=='q':
            cleanQuit()
            return None
        else: #otherwise if input exists try to parse it
            try:
                result = int(i)
            except:
                #if input is not a valid int
                print("invalid input, please try again")
                continue
            #if input not in the range
            if result not in allowed_range:
                print("outside allowed range, please try again")
                continue
            #otherwise return it
            else:
                return result

def get_float_num_from_user(item_str, lower_lim, upper_lim):
    result = None
    while True:
        #ask for input
        i = input("Press q+Enter to quit, or enter " + item_str + "...")
        #exit the function and cleanQuit
        if i=='q':
            cleanQuit()
            return None
        else: #otherwise if input exists try to parse it
            try:
                result = float(i)
            except:
                #if input is not a valid int
                print("invalid input, please try again")
                continue
            #if input not in the range
            if result < lower_lim or result > upper_lim:
                print("outside allowed range, please try again")
                continue
            #otherwise return it
            else:
                return result

def get_bool_from_user(item_str):
    while True:
        #ask for input
        i = input("Press q+Enter to quit, or enter t/f for bool " + item_str + "...")
        #exit the function and cleanQuit
        if i=='q':
            cleanQuit()
            return None
        else: #otherwise if input exists try to parse it
            if i =='t':
                return True
            if i =='f':
                return False
            else:
                print("invalid input, please try again")
                continue




signal.signal(signal.SIGINT, cleanQuitInt)


main()
'''
for leg in range(len(odrvs)):
    for joint in range(len(odrvs[0])):
        if odrvs[leg][joint] == None:
            continue
        xrlo.mixed_config_all(perm=True)
        #Note that mixed impedance control units are in radians already!
        odrvs[leg][joint].axis0.controller.config.pos_gain = 10*CPR2RAD #10 with no gearing is ok. 100 was violent. Need 450 to 900 max
        odrvs[leg][joint].axis1.controller.config.pos_gain = 10*CPR2RAD
        odrvs[leg][joint].axis0.controller.config.vel_gain = 0.75*CPR2RAD #1 with no gearing causes light shakes = 48/15 = 3.2Nms/rad per motor at joint. Nice! Need 2.177 overall
        odrvs[leg][joint].axis1.controller.config.vel_gain = 0.75*CPR2RAD
        odrvs[leg][joint].axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        odrvs[leg][joint].axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

'''


#odrive.utils.start_liveplotter(lambda:[(odrvs[leg][joint].axis0.encoder.pos_estimate,odrvs[leg][joint].axis0.encoder.pos_estimate) for leg in range(len(odrvs)) for joint in range(len(odrvs[0]))])
#start_liveplotter(lambda:[odrv0.axis0.encoder.pos_estimate, odrv0.axis1.encoder.pos_estimate,odrv1.axis0.encoder.pos_estimate, odrv1.axis1.encoder.pos_estimate,odrv2.axis0.encoder.pos_estimate, odrv2.axis1.encoder.pos_estimate,odrv3.axis0.encoder.pos_estimate, odrv3.axis1.encoder.pos_estimate,odrv4.axis0.encoder.pos_estimate, odrv4.axis1.encoder.pos_estimate,odrv5.axis0.encoder.pos_estimate, odrv5.axis1.encoder.pos_estimate])

print("Done with program.")
