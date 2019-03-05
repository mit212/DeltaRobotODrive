#!/usr/bin/python
"""
Interface from Python to ODrive

Daniel J. Gonzalez - dgonz@mit.edu
2.12 Intro to Robotics Spring 2019
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

pi = 3.1415927
in2mm = 25.4
mm2in = 1/in2mm
in2m = in2mm/1000

Nm2A = 0.00000604 #N*m/radian to A/count
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

CPR2RAD = (2*math.pi/400000)

odrvs = [None, None]
'''[[ODrive 0, ODrive 1]]'''
usb_serials = ['2061377C3548', '208637853548']
axes = [None, None, None]
axis0 = None
axis1 = None
axis2 = None

def rad2Count(angle):
    try:
        return [100000-ang/CPR2RAD for ang in angle]
    except TypeError:
        return 100000-angle/CPR2RAD

def r2c(angle):
    return rad2Count(angle)

def count2Rad(count):
    try:
        return [(100000-cnt)*CPR2RAD for cnt in count]
    except TypeError:
        return (100000-count)*CPR2RAD

def c2r(count):
    return count2Rad(count)

def print_controllers():
    for axis in axes:
        print(axis.controller)

def print_encoders():
    for axis in axes:
        print(axis.encoder)

def printErrorStates():
    ii = 0
    for axis in axes:
        print('axis',ii, ' axis error:',hex(axis.error))
        print('axis',ii, ' motor error:',hex(axis.motor.error))
        print('axis',ii, ' encoder error:',hex(axis.encoder.error))
        ii+=1

def printPos():
    ii = 0
    for axis in axes:
        print(ii, ' pos_estimate: ', axis.encoder.pos_estimate)
        print(ii, ' count_in_cpr: ', axis.encoder.count_in_cpr)
        print(ii, ' shadow_count: ', axis.encoder.shadow_count)
        ii+=1

def print_all():
    printErrorStates()
    print_encoders()
    print_controllers()

def connect_all():
    global axis0, axis1, axis2, axes
    for ii in range(len(odrvs)):
        if usb_serials[ii] == None:
            continue
        print("finding odrive: " + usb_serials[ii]+ "...")
        odrvs[ii]= odrive.find_any(serial_number = usb_serials[ii])
        print("found odrive! " + str(ii))
    axis0 = odrvs[0].axis0
    axis1 = odrvs[0].axis1
    axis2 = odrvs[1].axis0
    axes[0] = axis0
    axes[1] = axis1
    axes[2] = axis2

def reboot(ii):
    try:
        odrvs[ii].reboot()
    except:
        print('Rebooted ',ii)
    time.sleep(5)

def reboot_all():
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

connect_all()
printPos()

def vel_test_one(ii = 0, amt = 10000, mytime = 2):
    axis = axes[ii]
    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    axis.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
    
    axis.controller.vel_setpoint = 0
    time.sleep(mytime)
    print(0)
    print_all()
    time.sleep(0.25)
    
    axis.controller.vel_setpoint = amt
    time.sleep(mytime)
    print(1)
    print_all()
    time.sleep(0.25)

    axis.controller.vel_setpoint = 0
    time.sleep(mytime)
    print(2)
    print_all()
    time.sleep(0.25)

    axis.controller.vel_setpoint = -amt
    time.sleep(mytime)
    print(3)
    print_all() 
    time.sleep(0.25)

    axis.controller.vel_setpoint = 0
    time.sleep(mytime)
    print(4)
    print_all()

def vel_test_all(amt = 30000, mytime = 2):
    count = 0
    for axis in axes:
        axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        axis.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
        axis.controller.vel_setpoint = 0
        count += 1
        time.sleep(0.25)
        print(str(count) + " " + str(axis.encoder.vel_estimate))
    time.sleep(mytime)
    print(0)
    #print_all()
    time.sleep(0.25)
    count = 0

    for axis in axes:
        axis.controller.vel_setpoint = -amt
        print("setpoint " + str(axis.controller.vel_setpoint))
        print("command current " + str(axis.motor.current_control.Iq_setpoint))
        count += 1
        time.sleep(0.25)
        print(str(count) + " " + str(axis.encoder.vel_estimate))
    time.sleep(mytime)
    print(1)
    #print_all() 
    time.sleep(0.25)
    count = 0

    for axis in axes:
        axis.controller.vel_setpoint = 0
        print("setpoint " + str(axis.controller.vel_setpoint))
        print("command current " + str(axis.motor.current_control.Iq_setpoint))
        count += 1
        time.sleep(0.25)
        print(str(count) + " " + str(axis.encoder.vel_estimate))
    time.sleep(mytime)
    print(2)
    #print_all()
    time.sleep(0.25)
    count = 0

    for axis in axes:
        axis.controller.vel_setpoint = amt
        print("setpoint " + str(axis.controller.vel_setpoint))
        print("command current " + str(axis.motor.current_control.Iq_setpoint))
        count += 1
        time.sleep(0.25)
        print(str(count) + " " + str(axis.encoder.vel_estimate))
    time.sleep(mytime)
    print(3)
    #print_all() 
    time.sleep(0.25)
    count = 0

    for axis in axes:
        axis.controller.vel_setpoint = 0
        print("setpoint " + str(axis.controller.vel_setpoint))
        print("command current " + str(axis.motor.current_control.Iq_setpoint))
        count += 1
        time.sleep(0.25)
        print(str(count) + " " + str(axis.encoder.vel_estimate))
    time.sleep(mytime)
    print(4)
    #print_all()

def trajMoveCnt(posDesired = (10000, 10000, 10000), velDesired = 50000, accDesired = 50000):
    for ii in range(0,3):
        axis = axes[ii]
        axis.trap_traj.config.vel_limit = velDesired #600000 max, 50000 is 1/8 rev per second
        axis.trap_traj.config.accel_limit = accDesired #50000 is 1/8 rev per second per second
        axis.trap_traj.config.decel_limit = accDesired
        axis.controller.move_to_pos(posDesired[ii]) 

def trajMoveRad(posDesired = (0, 0, 0), velDesired = 2*pi/8, accDesired = 2*pi/8):
    trajMoveCnt(rad2Count(posDesired), rad2Count(velDesired), rad2Count(accDesired))

def test_one(ii = 0, amt = 10000, mytime = 5):
    axis = axes[ii]
    # IF WE ARE USING INDEX, START AT IDLE, THEN CHANGE TO CLOSED LOOP!!!!
    axis.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    axis.controller.pos_setpoint = amt
    time.sleep(mytime)
    print(0)
    print_all()
    time.sleep(0.25)

    axis.controller.pos_setpoint = 0
    time.sleep(mytime)
    print(1)
    print_all()
    time.sleep(0.25)

    axis.controller.pos_setpoint = amt
    time.sleep(mytime)
    print(2)
    print_all() 
    time.sleep(0.25)

    axis.controller.pos_setpoint = 0
    time.sleep(mytime)
    print(3)
    print_all()


def test_all(amt = 50000, mytime = 2):
    for axis in axes:
        # IF WE ARE USING INDEX, START AT IDLE, THEN CHANGE TO CLOSED LOOP!!!!
        axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        axis.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
        axis.controller.pos_setpoint = amt
    time.sleep(mytime)
    print(0)
    #print_all()
    time.sleep(0.25)

    for axis in axes:
        axis.controller.pos_setpoint = 0
        print("setpoint " + str(axis.controller.pos_setpoint))
        print("command current " + str(axis.motor.current_control.Iq_setpoint))
        print("detect current " + str(axis.motor.current_control.Iq_measured))
    time.sleep(mytime)
    print(1)
    #print_all()
    time.sleep(0.25)

    for axis in axes:
        axis.controller.pos_setpoint = amt
        print("setpoint " + str(axis.controller.pos_setpoint))
        print("command current " + str(axis.motor.current_control.Iq_setpoint))
        print("detect current " + str(axis.motor.current_control.Iq_measured))
    time.sleep(mytime)
    print(2)
    #print_all()
    time.sleep(0.25)

    for axis in axes:
        axis.controller.pos_setpoint = 0
        print("setpoint " + str(axis.controller.pos_setpoint))
        print("command current " + str(axis.motor.current_control.Iq_setpoint))
        print("detect current " + str(axis.motor.current_control.Iq_measured))
    time.sleep(mytime)
    print(3)
    #for axis in axes:
    #    axis.requested_state = AXIS_STATE_IDLE
    #print_all()
    time.sleep(0.25)
    #axis.requested_state = AXIS_STATE_IDLE
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
        if(reset):
            axis.motor.config.pre_calibrated = False
            axis.encoder.config.pre_calibrated = False

        #motor current limit
        axis.motor.config.current_lim = 5

        #pole pairs
        axis.motor.config.pole_pairs = 4

        axis.controller.config.vel_limit = 600000 #50000 counts/second is 1/8 revolution per second

        # 0.0612 [(revolutions/second)/Volt], 400000 counts per revolution
        # Max speed is 1.35 Revolutions/second, or 539000counts/second
        axis.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
        axis.encoder.config.cpr = 4000
        axis.encoder.config.bandwidth = 1000
        axis.encoder.config.use_index = True
        axis.encoder.config.zero_count_on_find_idx = True
        axis.encoder.config.idx_search_speed = 1
        axis.encoder.config.pre_calibrated = False

        #motor calibration current
        axis.motor.config.calibration_current = 5

        #axis state
        if(axis.motor.config.pre_calibrated == False):
            axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    print("Done doing setup.")
    time.sleep(20)
    print("Saving Configuration...")
    for axis in axes:
        axis.motor.config.pre_calibrated = True
        axis.config.startup_encoder_index_search = True
        axis.config.startup_encoder_offset_calibration = True
        axis.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL

        #motor calibration current FOR INDEX SEARCH
        axis.motor.config.calibration_current = 5

        #Set closed loop gains
        kP_des = Nm2A*100 # pos_gain 2
        kD_des = Nm2A*50  # vel_gain 0.0015 / 5

        axis.controller.config.pos_gain = kP_des/kD_des #Convert to Cascaded Gain Structure
        #https://github.com/madcowswe/ODrive/blob/451e79519637fdcf33f220f7dae9a28b15e014ba/Firmware/MotorControl/controller.cpp#L151
        axis.controller.config.vel_gain = kD_des
        axis.controller.config.vel_integrator_gain = 0
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

def get_cnt_all():
    positions = [None,None,None]
    ii = 0
    for axis in axes:
        positions[ii] = axis.encoder.pos_estimate
        ii+=1
    return positions

def get_rad_all():
    return count2Rad(get_cnt_all())