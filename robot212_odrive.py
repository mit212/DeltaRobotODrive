
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

odrvs = [[None, None]]
'''[[ODrive 0, ODrive 1]]'''
usb_serials = [['2087377B3548', '208637853548']]

def print_controllers():
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            if odrvs[leg][joint] == None:
                continue
            print(odrvs[leg][joint].axis0.controller)
            print(odrvs[leg][joint].axis1.controller)

def print_encoders():
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            if odrvs[leg][joint] == None:
                continue
            print(odrvs[leg][joint].axis0.encoder)
            print(odrvs[leg][joint].axis1.encoder)

def printErrorStates():
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            if odrvs[leg][joint] == None:
                continue
            print('leg',leg, ' joint',joint, ' axis0 error:',hex(odrvs[leg][joint].axis0.error))
            print('leg',leg, ' joint',joint, ' axis1 error:',hex(odrvs[leg][joint].axis1.error))
            print('leg',leg, ' joint',joint, ' motor0 error:',hex(odrvs[leg][joint].axis0.motor.error))
            print('leg',leg, ' joint',joint, ' motor1 error:',hex(odrvs[leg][joint].axis1.motor.error))
            print('leg',leg, ' joint',joint, ' encoder0 error:',hex(odrvs[leg][joint].axis0.encoder.error))
            print('leg',leg, ' joint',joint, ' encoder1 error:',hex(odrvs[leg][joint].axis1.encoder.error))
def print_all():
    printErrorStates()
    print_encoders()
    print_controllers()

def connect_all():
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            if usb_serials[leg][joint] == None:
                continue
            print("finding odrive: " + usb_serials[leg][joint] + "...")
            odrvs[leg][joint] = odrive.find_any(serial_number = usb_serials[leg][joint])
            print("found odrive! leg: " + str(leg) + ", joint: " + str(joint))
connect_all()

def test_all(amt = 100000, mytime = 5):
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            odrvs[leg][joint].axis0.controller.pos_setpoint = 0
            odrvs[leg][joint].axis1.controller.pos_setpoint = 0
    print_all()
    time.sleep(mytime)
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            odrvs[leg][joint].axis0.controller.pos_setpoint = amt
            odrvs[leg][joint].axis1.controller.pos_setpoint = amt
    print_all()
    time.sleep(mytime)
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            odrvs[leg][joint].axis0.controller.pos_setpoint = 0
            odrvs[leg][joint].axis1.controller.pos_setpoint = 0
    print_all()
    time.sleep(mytime)
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            odrvs[leg][joint].axis0.controller.pos_setpoint = amt
            odrvs[leg][joint].axis1.controller.pos_setpoint = amt
    print_all()
    time.sleep(mytime)
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            odrvs[leg][joint].axis0.controller.pos_setpoint = 0
            odrvs[leg][joint].axis1.controller.pos_setpoint = 0
#test_all()

def set_gains(k_p, k_d, perm = True):
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            if(k_d != 0):
                odrvs[leg][joint].axis0.controller.config.pos_gain = Nm2A*k_p/k_d
                odrvs[leg][joint].axis1.controller.config.pos_gain = Nm2A*k_p/k_d
                odrvs[leg][joint].axis0.controller.config.vel_gain = k_d*Nm2A
                odrvs[leg][joint].axis1.controller.config.vel_gain = k_d*Nm2A
                if(perm):
                    odrvs[leg][joint].save_configuration()
                    time.sleep(2)
    

def set_gainsCounts(k_p, k_d):
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            odrvs[leg][joint].axis0.controller.config.pos_gain = k_p
            odrvs[leg][joint].axis1.controller.config.pos_gain = k_p
            odrvs[leg][joint].axis0.controller.config.vel_gain = k_d
            odrvs[leg][joint].axis1.controller.config.vel_gain = k_d
            odrvs[leg][joint].save_configuration()
    time.sleep(2)

def full_init(reset = True):
    if(reset):
        for leg in range(len(odrvs)):
            for joint in range(len(odrvs[0])):
                if odrvs[leg][joint] == None:
                    continue
                odrvs[leg][joint].axis0.motor.config.pre_calibrated = False
                odrvs[leg][joint].axis0.encoder.config.pre_calibrated = False
                odrvs[leg][joint].axis1.motor.config.pre_calibrated = False
                odrvs[leg][joint].axis1.encoder.config.pre_calibrated = False
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            if odrvs[leg][joint] == None:
                continue
            #motor current limit
            odrvs[leg][joint].axis0.motor.config.current_lim = 4
            odrvs[leg][joint].axis1.motor.config.current_lim = 4

            #pole pairs
            odrvs[leg][joint].axis0.motor.config.pole_pairs = 4
            odrvs[leg][joint].axis1.motor.config.pole_pairs = 4

            odrvs[leg][joint].axis0.controller.config.vel_limit = 600000 #50000 counts/second is 1/8 revolution per second
            odrvs[leg][joint].axis1.controller.config.vel_limit = 600000 
            # 0.0612 [(revolutions/second)/Volt], 400000 counts per revolution
            # Max speed is 1.35 Revolutions/second, or 539000counts/second
            odrvs[leg][joint].axis0.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
            odrvs[leg][joint].axis1.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
            odrvs[leg][joint].axis0.encoder.config.cpr = 4000
            odrvs[leg][joint].axis1.encoder.config.cpr = 4000
            odrvs[leg][joint].axis0.encoder.config.use_index = True 
            odrvs[leg][joint].axis1.encoder.config.use_index = True

            #motor calibration current
            odrvs[leg][joint].axis0.motor.config.calibration_current = 4
            odrvs[leg][joint].axis1.motor.config.calibration_current = 4

            #brake resistance
            odrvs[leg][joint].config.brake_resistance = 0

            # Change velocity pll bandwidth high temporarily to ensure calibration works well
            #odrvs[leg][joint].axis0.encoder.set_pll_bandwidth(1570)
            #odrvs[leg][joint].axis1.encoder.set_pll_bandwidth(1570)

            #axis state
            if(odrvs[leg][joint].axis0.motor.config.pre_calibrated == False):
                odrvs[leg][joint].axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            if(odrvs[leg][joint].axis1.motor.config.pre_calibrated == False):
                odrvs[leg][joint].axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    print("Done doing setup.")
    time.sleep(20)
    print("Saving Configuration...")
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            errorFlag = 0
            if odrvs[leg][joint] == None:
                continue
            odrvs[leg][joint].axis0.motor.config.pre_calibrated = True
            odrvs[leg][joint].axis1.motor.config.pre_calibrated = True
            odrvs[leg][joint].axis0.encoder.config.pre_calibrated = True
            odrvs[leg][joint].axis0.config.startup_encoder_index_search = True
            odrvs[leg][joint].axis1.encoder.config.pre_calibrated = True
            odrvs[leg][joint].axis1.config.startup_encoder_index_search = True
            
            #should always be default on all - use these if gets screwed up ever
            #odrvs[leg][joint].axis0.controller.set_current_controller_bandwidth(157)
            #odrvs[leg][joint].axis1.controller.set_current_controller_bandwidth(157)

            #Set closed loop gains
            kP_des = Nm2A*0.25 #2 Nm/rad
            kD_des = Nm2A*1

            odrvs[leg][joint].axis0.controller.config.pos_gain = kP_des/kD_des #Convert to Cascaded Gain Structure
            odrvs[leg][joint].axis1.controller.config.pos_gain = kP_des/kD_des
            #https://github.com/madcowswe/ODrive/blob/451e79519637fdcf33f220f7dae9a28b15e014ba/Firmware/MotorControl/controller.cpp#L151
            odrvs[leg][joint].axis0.controller.config.vel_gain = kD_des
            odrvs[leg][joint].axis1.controller.config.vel_gain = kD_des
            odrvs[leg][joint].axis0.controller.pos_setpoint = 0
            odrvs[leg][joint].axis1.controller.pos_setpoint = 0

            #axis state
            #odrvs[leg][joint].axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            odrvs[leg][joint].axis0.config.startup_closed_loop_control = True
            #odrvs[leg][joint].axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            odrvs[leg][joint].axis1.config.startup_closed_loop_control = True
            # save configuration
            odrvs[leg][joint].save_configuration()
            time.sleep(2)
            printErrorStates()
    try:
        odrvs[0][0].reboot()
    except:
        print('Rebooted 0')
    try:
        odrvs[0][1].reboot()
    except:
        print('Rebooted 1')
    time.sleep(5)
    print("Done initializing! Reconnecting...")
    connect_all()

def make_perm(leg, joint):
    odrvs[leg][joint].save_configuration()

def make_perm_all():
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            if odrvs[leg][joint] == None:
                continue
            odrvs[leg][joint].save_configuration()

def closed_loop_state_all(perm=False):
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            if odrvs[leg][joint] == None:
                continue
            odrvs[leg][joint].axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            odrvs[leg][joint].axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            if(perm):
                odrvs[leg][joint].save_configuration()


def set_pll(leg, joint, pll_bandwidth, perm =False):
    odrvs[leg][joint].axis0.encoder.set_pll_bandwidth(pll_bandwidth)
    odrvs[leg][joint].axis1.encoder.set_pll_bandwidth(pll_bandwidth)
    if(perm):
        odrvs[leg][joint].save_configuration()

def reboot(leg, joint):
    odrvs[leg][joint].reboot()

def reboot_all():
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            if odrvs[leg][joint] == None:
                continue
            odrvs[leg][joint].reboot()

def get_pos_all():
    positions = [[None,None,None],[None,None,None]]
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            if odrvs[leg][joint] == None:
                continue
            positions[leg][joint] = [odrvs[leg][joint].axis0.encoder.pos_estimate, odrvs[leg][joint].axis1.encoder.pos_estimate]
    return positions