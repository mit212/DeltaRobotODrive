
"""
Created on Thu Jul 18 15:19:02 2018

@author: beeman
"""


"""
Modified on Feb 14, 2019

@author: dgonz
"""

"""
helpful odrive functions for xrl state machine
"""

import odrive
from odrive.enums import *
import time
import math
import fibre

import serial
import struct
import signal
import sys
#import xrl_kinematics as xrlk
from dataLogger import *

in2mm = 25.4
mm2in = 1/in2mm
in2m = in2mm/1000

Nm2A = 0.00000604#N*m/radian to A/count
#https://www.wolframalpha.com/input/?i=(1+N*m%2Fradian)*(2*pi+radians%2F400000)*(1%2F(2.6+N*m%2FA))

zeroVec = [[[0,0],[0,0]]]
offsets = [[[-8.59,-6.11],[-3.61,5.89]]]
thtDesired = [[[0,0],[0,0]]]
velDesired = [[[0,0],[0,0]]]
kP = [[[0,0],[0,0]]]
kD = [[[0,0],[0,0]]]
home_kp = [[[0,0],[0,0]]]
home_kd = [[[0,0],[0,0]]]
kPd = [[[0,0],[0,0]]]
kDd = [[[0,0],[0,0]]]

gear_ratios = [32.0/15.0, 48.0/15.0, 36.0/15.0]
CPR2RAD = (2*math.pi/16384.0)
thtActual = [[[0,0],[0,0],[0,0]],[[0,0], [0,0], [0,0]]]
velActual =   [[[0,0],[0,0],[0,0]],[[0,0], [0,0], [0,0]]]
curCommand = [[[0,0],[0,0],[0,0]],[[0,0], [0,0], [0,0]]]
myLogger = dataLogger('data.txt')

odrvs = [None, None]
'''[[ODrive 0, ODrive 1]]'''
usb_serials = ['2087377B3548', '208637853548']
axes = [None, None, None]

def print_controllers():
    for axis in axes:
        print(axis.controller)

def print_encoders():
    for axis in axes:
        print(axis.encoder)

def printErrorStates():
    ii = 0
    for axis in axes:
        print(axis.controller)
        print('axis',ii, ' axis error:',hex(axis.error))
        print('axis',ii, ' motor error:',hex(axis.motor.error))
        print('axis',ii, ' encoder error:',hex(axis.encoder.error))
        ii+=1
def print_all():
    printErrorStates()
    print_encoders()
    print_controllers()

def connect_all():
    for ii in range(len(odrvs)):
        if usb_serials[ii] == None:
            continue
        print("finding odrive: " + usb_serials[ii]+ "...")
        odrvs[ii]= odrive.find_any(serial_number = usb_serials[ii])
        print("found odrive! " + str(ii))
    axes[0] = odrvs[0].axis0
    axes[1] = odrvs[0].axis1
    axes[2] = odrvs[1].axis0

connect_all()

def test_all(amt = 100000, mytime = 5):
    for axis in axes:
        axis.controller.pos_setpoint = 0
    print_all()
    time.sleep(mytime)

    for axis in axes:
        axis.controller.pos_setpoint = amt
    print_all()
    time.sleep(mytime)

    for axis in axes:
        axis.controller.pos_setpoint = 0
    print_all()
    time.sleep(mytime)   

    for axis in axes:
        axis.controller.pos_setpoint = 0
    time.sleep(mytime)
#test_all()

def set_gains(k_p, k_d, perm = True):
    for axis in axes:
        if(k_d != 0):
            axis.controller.config.pos_gain = Nm2A*k_p/k_d
            axis.controller.config.vel_gain = k_d*Nm2A
    if(perm):
        odrvs[0].save_configuration()
        odrvs[1].save_configuration()
        time.sleep(2)
    

def set_gainsCounts(k_p, k_d,perm = True):
    for axis in axes:
        if(k_d != 0):
            axis.controller.config.pos_gain = k_p
            axis.controller.config.vel_gain = k_d
    if(perm):
        odrvs[0].save_configuration()
        odrvs[1].save_configuration()
        time.sleep(2)

def full_init(reset = True):
    #brake resistance
    odrvs[0].config.brake_resistance = 0
    odrvs[1].config.brake_resistance = 0

    for axis in axes:
        if odrvs[leg][joint] == None:
            continue
        if(reset):
            axis.motor.config.pre_calibrated = False
            axis.encoder.config.pre_calibrated = False

        #motor current limit
        axis.motor.config.current_lim = 4

        #pole pairs
        axis.motor.config.pole_pairs = 4

        axis.controller.config.vel_limit = 600000 #50000 counts/second is 1/8 revolution per second

        # 0.0612 [(revolutions/second)/Volt], 400000 counts per revolution
        # Max speed is 1.35 Revolutions/second, or 539000counts/second
        axis.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
        axis.encoder.config.cpr = 4000
        axis.encoder.config.use_index = True

        #motor calibration current
        axis.motor.config.calibration_current = 2

        #axis state
        if(axis.motor.config.pre_calibrated == False):
            axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    print("Done doing setup.")
    time.sleep(20)
    print("Saving Configuration...")
    for axis in axes:
        errorFlag = 0
        if odrvs[leg][joint] == None:
            continue
        axis.motor.config.pre_calibrated = True
        axis.encoder.config.pre_calibrated = True
        axis.config.startup_encoder_index_search = True

        #motor calibration current FOR INDEX SEARCH
        axis.motor.config.calibration_current = 0.5

        #Set closed loop gains
        kP_des = Nm2A*0.25 #2 Nm/rad
        kD_des = Nm2A*1

        axis.controller.config.pos_gain = kP_des/kD_des #Convert to Cascaded Gain Structure
        #https://github.com/madcowswe/ODrive/blob/451e79519637fdcf33f220f7dae9a28b15e014ba/Firmware/MotorControl/controller.cpp#L151
        axis.controller.config.vel_gain = kD_des
        axis.controller.pos_setpoint = 0

        #axis state
        #odrvs[leg][joint].axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        axis.config.startup_closed_loop_control = True
    # save configuration
    odrvs[0].save_configuration()
    odrvs[1].save_configuration()
    time.sleep(2)
    printErrorStates()
    try:
        odrvs[0].reboot()
    except:
        print('Rebooted 0')
    try:
        odrvs[1].reboot()
    except:
        print('Rebooted 1')
    time.sleep(5)
    print("Done initializing! Reconnecting...")
    connect_all()

def make_perm(ii):
    odrvs[ii].save_configuration()

def make_perm_all():
    for ii in range(len(odrvs)):
        if odrvs[ii] == None:
            continue
        odrvs[ii].save_configuration()

def closed_loop_state_all():
    for axis in axes:
        axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

def reboot(ii):
    odrvs[ii].reboot()

def reboot_all():
    for ii in range(len(odrvs)):
        if odrvs[leg][joint] == None:
            continue
        odrvs[ii].reboot()

def get_pos_all():
    positions = [None,None,None]
    ii = 0
    for axis in axes:
        ii+=1
        positions[ii] = axis.encoder.pos_estimate
        ii+=1
    return positions