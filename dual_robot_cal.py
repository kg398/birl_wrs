from urx import urx
import time
import copy
import serial
import math
import scipy.optimize
import numpy as np

# ---will only work with windows--- (only used for manual calibration)
import msvcrt

import waypoints as wp

def cal_dual_robots(burt,urnie):
    urnie.set_tcp(wp.cal_rotary_tcp)

    burt.movej(wp.burt_passj,wait=False)
    urnie.movej(wp.urnie_passj,wait=False)

    demand_pose = urnie.calc_pose(burt.getl())
    demand_pose[0]+=-0.01
    urnie.translatejl(demand_pose)
    urnie_posa = keyboard_control(urnie)
    burt_posa = burt.getl()

    urnie.translatejl_rel([-0.15,0,0,0,0,0])
    burt.translatejl_rel([-0.1,-0.15,0.05,0,0,0])

    demand_pose = urnie.calc_pose(burt.getl())
    demand_pose[0]+=-0.01
    urnie.translatejl(demand_pose)
    urnie_posb = keyboard_control(urnie)
    burt_posb = burt.getl()

    urnie.translatejl_rel([-0.15,0,0,0,0,0])
    burt.translatejl_rel([0.3,0.05,0.05,0,0,0])

    demand_pose = urnie.calc_pose(burt.getl())
    demand_pose[0]+=-0.02
    urnie.translatejl(demand_pose)
    urnie_posc = keyboard_control(urnie)
    burt_posc = burt.getl()

    urnie.translatejl_rel([-0.15,0,0,0,0,0])
    burt.home(wait=False)
    urnie.home(wait=False)

    print("dual_rob_cal = [{},   # burt pos-a".format(burt_posa))
    print("                {},   # burt pos-b".format(burt_posb))
    print("                {},   # burt pos-c".format(burt_posc))
    print("                {},   # urnie pos-a".format(urnie_posa))
    print("                {},   # urnie pos-b".format(urnie_posb))
    print("                {}]   # urnie pos-c".format(urnie_posc))
    
    urnie.set_tcp(wp.rotary_tcp)
    return

def keyboard_control(urnie):
    print("Starting manual point locating, press 'e' to end\nmove in x: 'w' and 's'\nmove in y: 'a' and 'd'\nmove in z: 'r' and 'l'")
    ipt = 'q'
    while ipt!='e':
        ipt = bytes.decode(msvcrt.getch())
        #print('\n'+ipt)
        if ipt=='w':
            urnie.translatel_rel([0.0005,0,0,0,0,0])
        if ipt=='s':
            urnie.translatel_rel([-0.0005,0,0,0,0,0])
        if ipt=='d':
            urnie.translatel_rel([0,-0.0005,0,0,0,0])
        if ipt=='a':
            urnie.translatel_rel([0,0.0005,0,0,0,0])
        if ipt=='r':
            urnie.translatel_rel([0,0,0.0005,0,0,0])
        if ipt=='l':
            urnie.translatel_rel([0,0,-0.0005,0,0,0])
    print("Location confirmed: {}\n".format(urnie.getl()))
    return urnie.getl()
