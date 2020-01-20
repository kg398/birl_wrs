import time
import copy
from math import pi
import math
import numpy as np
import scipy
import _thread
import random

import waypoints as wp
import dual_robot_cal as drc

# thread flags
urnie_flag = False
burt_flag = False

class assembly():
    def __init__(self):
        self.offset = np.array([wp.ac_tr[0],wp.ac_tr[1],math.asin((wp.ac_br[0]-wp.ac_tr[0])/(wp.ac_br[1]-wp.ac_tr[1]))])
        self.right_tray_offset = np.array([wp.ac_right_tray_tr[0],wp.ac_right_tray_tr[1],math.asin((wp.ac_right_tray_br[0]-wp.ac_right_tray_tr[0])/(wp.ac_right_tray_br[1]-wp.ac_right_tray_tr[1]))])
        self.left_tray_offset = np.array([wp.ac_left_tray_rel[0],wp.ac_left_tray_rel[1],0]) + self.right_tray_offset
        return

    def calc_pose(self,assembly_pose):
        x = self.offset[0] + assembly_pose[0]*np.cos(self.offset[2]) + assembly_pose[1]*np.sin(self.offset[2])
        y = self.offset[1] - assembly_pose[0]*np.sin(self.offset[2]) + assembly_pose[1]*np.cos(self.offset[2])

        dz_x = (wp.ac_tl[2]-wp.ac_tr[2])/math.sqrt((wp.ac_tl[0]-wp.ac_tr[0])*(wp.ac_tl[0]-wp.ac_tr[0])+(wp.ac_tl[1]-wp.ac_tr[1])*(wp.ac_tl[1]-wp.ac_tr[1]))
        dz_y = (wp.ac_br[2]-wp.ac_tr[2])/math.sqrt((wp.ac_br[0]-wp.ac_tr[0])*(wp.ac_br[0]-wp.ac_tr[0])+(wp.ac_br[1]-wp.ac_tr[1])*(wp.ac_br[1]-wp.ac_tr[1]))
        z = assembly_pose[2] + wp.ac_tr[2] + assembly_pose[0]*dz_x + assembly_pose[1]*dz_y
        return [x,y,z,assembly_pose[3],assembly_pose[4],assembly_pose[5]]

    def right_tray(self,pose):
        x = self.right_tray_offset[0] + pose[0]*np.cos(self.right_tray_offset[2]) + pose[1]*np.sin(self.right_tray_offset[2])
        y = self.right_tray_offset[1] - pose[0]*np.sin(self.right_tray_offset[2]) + pose[1]*np.cos(self.right_tray_offset[2])

        dz_x = (wp.ac_right_tray_bl[2]-wp.ac_right_tray_br[2])/math.sqrt((wp.ac_right_tray_bl[0]-wp.ac_right_tray_br[0])*(wp.ac_right_tray_bl[0]-wp.ac_right_tray_br[0])+(wp.ac_right_tray_bl[1]-wp.ac_right_tray_br[1])*(wp.ac_right_tray_bl[1]-wp.ac_right_tray_br[1]))
        dz_y = (wp.ac_right_tray_br[2]-wp.ac_right_tray_tr[2])/math.sqrt((wp.ac_right_tray_br[0]-wp.ac_right_tray_tr[0])*(wp.ac_right_tray_br[0]-wp.ac_right_tray_tr[0])+(wp.ac_right_tray_br[1]-wp.ac_right_tray_tr[1])*(wp.ac_right_tray_br[1]-wp.ac_right_tray_tr[1]))
        z = pose[2] + wp.ac_right_tray_tr[2] + pose[0]*dz_x + pose[1]*dz_y
        return [x,y,z,pose[3],pose[4],pose[5]]

    def left_tray(self,pose):
        x = self.left_tray_offset[0] + pose[0]*np.cos(self.left_tray_offset[2]) + pose[1]*np.sin(self.left_tray_offset[2])
        y = self.left_tray_offset[1] - pose[0]*np.sin(self.left_tray_offset[2]) + pose[1]*np.cos(self.left_tray_offset[2])

        dz_x = (wp.ac_right_tray_bl[2]-wp.ac_right_tray_br[2])/math.sqrt((wp.ac_right_tray_bl[0]-wp.ac_right_tray_br[0])*(wp.ac_right_tray_bl[0]-wp.ac_right_tray_br[0])+(wp.ac_right_tray_bl[1]-wp.ac_right_tray_br[1])*(wp.ac_right_tray_bl[1]-wp.ac_right_tray_br[1]))
        dz_y = (wp.ac_right_tray_br[2]-wp.ac_right_tray_tr[2])/math.sqrt((wp.ac_right_tray_br[0]-wp.ac_right_tray_tr[0])*(wp.ac_right_tray_br[0]-wp.ac_right_tray_tr[0])+(wp.ac_right_tray_br[1]-wp.ac_right_tray_tr[1])*(wp.ac_right_tray_br[1]-wp.ac_right_tray_tr[1]))
        z = pose[2] + wp.ac_right_tray_tr[2]+wp.ac_left_tray_rel[2] + pose[0]*dz_x + pose[1]*dz_y
        return [x,y,z,pose[3],pose[4],pose[5]]


def calibrate(urnie):
    urnie.cal_fingers()
    urnie.set_tcp(wp.cal_rotary_tcp)

    demand_pose = copy.deepcopy(wp.ac_tl)
    demand_pose[2]+=0.01
    urnie.movejl(demand_pose)
    ac_tl1 = drc.keyboard_control(urnie)
    urnie.home()

    demand_pose = copy.deepcopy(wp.ac_tr)
    demand_pose[2]+=0.01
    urnie.movejl(demand_pose)
    ac_tr1 = drc.keyboard_control(urnie)
    urnie.home()

    demand_pose = copy.deepcopy(wp.ac_br)
    demand_pose[2]+=0.01
    urnie.movejl(demand_pose)
    ac_br1 = drc.keyboard_control(urnie)
    urnie.home()
    
    print("ac_tl =",ac_tl1)
    print("ac_tr =",ac_tr1)
    print("ac_br =",ac_br1)

    urnie.set_tcp(wp.rotary_tcp)
    return

def cal_trays(urnie):
    urnie.set_tcp(wp.cal_rotary_tcp)

    demand_pose = copy.deepcopy(wp.ac_right_tray_tr)
    demand_pose[2]+=0.01
    urnie.movejl(demand_pose)
    right_tray_tr = drc.keyboard_control(urnie)
    urnie.home()

    demand_pose = copy.deepcopy(wp.ac_right_tray_br)
    demand_pose[2]+=0.01
    urnie.movejl(demand_pose)
    right_tray_br = drc.keyboard_control(urnie)
    urnie.home()

    demand_pose = copy.deepcopy(wp.ac_right_tray_bl)
    demand_pose[2]+=0.01
    urnie.movejl(demand_pose)
    right_tray_bl = drc.keyboard_control(urnie)
    urnie.home()

    demand_pose = copy.deepcopy(wp.ac_right_tray_bl)
    demand_pose[0]+=wp.ac_left_tray_rel[0]
    demand_pose[1]+=wp.ac_left_tray_rel[1]
    demand_pose[2]+=wp.ac_left_tray_rel[2]+0.01
    urnie.movejl(demand_pose)
    left_tray_rel = drc.keyboard_control(urnie)
    left_tray_rel[0]-=right_tray_bl[0]
    left_tray_rel[1]-=right_tray_bl[1]
    left_tray_rel[2]-=right_tray_bl[2]
    urnie.home()
    
    print("ac_right_tray_tr =",right_tray_tr)
    print("ac_right_tray_br =",right_tray_br)
    print("ac_right_tray_bl =",right_tray_bl)
    print("ac_left_tray_rel =",left_tray_rel)
    
    urnie.set_tcp(wp.rotary_tcp)
    return

def cal_allen_keys(urnie):
    urnie.set_tcp(wp.cal_rotary_tcp)

    demand_pose = copy.deepcopy(wp.ac_allen_m4_long)
    demand_pose[2]+=0.01
    urnie.movejl(demand_pose)
    m4_long = drc.keyboard_control(urnie)
    urnie.home()

    demand_pose = copy.deepcopy(wp.ac_allen_m4_short)
    demand_pose[2]+=0.01
    urnie.movejl(demand_pose)
    m4_short = drc.keyboard_control(urnie)
    urnie.home()

    demand_pose = copy.deepcopy(wp.ac_allen_m3)
    demand_pose[2]+=0.01
    urnie.movejl(demand_pose)
    m3 = drc.keyboard_control(urnie)
    urnie.home()

    demand_pose = copy.deepcopy(wp.ac_allen_m2)
    demand_pose[2]+=0.01
    urnie.movejl(demand_pose)
    m2 = drc.keyboard_control(urnie)
    urnie.home()
    
    print("ac_allen_m4_long =",m4_long)
    print("ac_allen_m4_short =",m4_short)
    print("ac_allen_m3 =",m3)
    print("ac_allen_m2 =",m2)
    
    urnie.set_tcp(wp.rotary_tcp)
    return

def get_allen_key(urnie,key,grease):
    global urnie_flag
    urnie_flag=True
    # open and orient fingers
    urnie.cal_gripper(wait=False)

    # home
    urnie.home()

    dz = -0.025
    # move above allen key
    if key == 'm4_long':
        urnie.movejl(wp.ac_allen_m4_long)
    elif key == 'm4_short':
        urnie.movejl(wp.ac_allen_m4_short)
        dz = -0.015
    elif key == 'm3':
        urnie.movejl(wp.ac_allen_m3)
        dz = -0.015
    elif key == 'm2':
        urnie.movejl(wp.ac_allen_m2)
        dz = -0.01

    # pick up key
    urnie.wait_for_gripper()
    print("did i stop here?")
    urnie.translatel_rel([0,0,dz])
    urnie.close_gripper(1)
    urnie.close_gripper(5)

    # move up and set key
    urnie.translatel_rel([0,0,0.06])
    urnie.translatel_rel([0,0.01,0])
    urnie.open_gripper(0)
    urnie.force_move([0, 0, -0.1],force=50)
    urnie.close_gripper(1,wait=False)
    urnie.translatel_rel([0,0,0.01])

    if grease==True:
        get_grease(urnie,key)

    # home
    urnie.home()
    urnie_flag=False
    return

def stow_allen_key(urnie,key):
    urnie_flag=True
    force = 50
    # home
    urnie.cal_fingers(wait=False)
    urnie.home()

    # move above allen key hole
    dz = -0.05
    if key == 'm4_long':
        demand_pose = copy.deepcopy(wp.ac_allen_m4_long)
    elif key == 'm4_short':
        demand_pose = copy.deepcopy(wp.ac_allen_m4_short)
        dz = -0.07
    elif key == 'm3':
        demand_pose = copy.deepcopy(wp.ac_allen_m3)
        force = 40
        dz = -0.07
    elif key == 'm2':
        demand_pose = copy.deepcopy(wp.ac_allen_m2)
        force = 30
    demand_pose[2]+=0.04
    urnie.movejl(demand_pose)

    # insert into hole
    urnie.insert(axis=[0,0,dz],max_force=force)
    urnie.open_gripper(1)
    urnie.open_gripper(5,wait=False)
    urnie.translatel_rel([0,0,0.03])

    # home
    urnie.home()
    urnie_flag=False
    return

def assemble_all(burt,urnie):
    # fixing plates
    subtask_f(burt,urnie,coords)
    subtask_g(burt,urnie,coords)

    # motor
    subtask_a(burt,urnie,coords)

    # motor pulley
    subtask_b(burt,urnie,coords)

    # output shaft
    stored_pose = subtask_c(burt,urnie,coords)

    # output pulley
    subtask_d(burt,urnie,coords,stored_pose)

    # belt
    subtask_h(burt,urnie,coords)

    # tensioner pulley
    subtask_e(burt,urnie,coords)
    return

def subtask_a(burt,urnie,coords):
    """
    # Target Parts: motor, m3 screws
    # Task: Assembling the motor to the motor fixing plate with the screws
    # Points: 4
    # Notes:
    """
    # init robots
    urnie.cal_fingers(wait=False)
    burt.cal_gripper(wait=False)

    # get allen key
    global urnie_flag
    _thread.start_new_thread(get_allen_key,(urnie,'m3',True))

    # move to motor
    burt.movejl(burt.urnie_to_burt(urnie.pedestal.left_tray([0.145,0.032,0.053,np.sqrt(pi*pi/2),np.sqrt(pi*pi/2),0])))
    burt.wait_for_gripper()
    burt.translatel_rel([0,0,-0.05])

    # pick up motor
    burt.close_gripper(1)
    burt.close_gripper(3)
    burt.open_gripper(1)
    burt.force_move([0,0,-0.03],force=40)
    burt.close_gripper(2)
    burt.translatel_rel([0,0,0.05])

    # move to motor orientor
    burt.set_tcp(wp.motor_centroid)
    burt.movej(wp.burt_motorj)
    burt.translatejl_pedestal([0.1865,0.008,0.04],robot=urnie)
    start_pose = burt.getl()
    burt.force_move([0,0,-0.035],force=30)
    end_pose = burt.getl()
    if start_pose[2]-end_pose[2]<0.01:
        burt.translatel_rel([0,0,0.005])
        burt.movel_tool([0,0,0,0,-30*pi/180.0,0])
        burt.force_move([0,0,-0.01],force=30)

    # orient motor
    burt.open_gripper(0)
    burt.open_gripper(1)
    burt.translatel_rel([0,0,0.01])

    if start_pose[2]-end_pose[2]<0.01:
        burt.movel_tool([0,0,0,0,30*pi/180.0,0])

    while urnie_flag == True:
        time.sleep(0.1)
    _thread.start_new_thread(get_screw,(urnie,'m3',[0.03,0.015]))

    stored_joints = burt.getj()
    burt.movel_tool([0,0,0,0,-60*pi/180.0,0])
    burt.close_gripper(2)
    burt.open_gripper(0)
    burt.open_gripper(0)
    burt.open_gripper(0)
    burt.movel_tool([0,0,0,0,60*pi/180.0,0])
    burt.movej(stored_joints)

    #burt.open_gripper(0)
    #burt.open_gripper(1)
    #burt.movel_tool([0,0,0,0,-60*pi/180.0,0])
    #burt.close_gripper(2)
    #burt.open_gripper(0)
    #burt.open_gripper(0)
    #burt.open_gripper(0)
    #burt.movel_tool([0,0,0,0,60*pi/180.0,0])
    #burt.movej(stored_joints)

    burt.open_gripper(1)
    burt.close_gripper(1)
    burt.force_move([0,0,-0.015],force=40)
    burt.close_gripper(1)
    burt.translatel_rel([0,0,0.05])
    burt.set_tcp(wp.pincher_tcp)

    # move to pedestal
    burt.home()
    
    #burt.set_tcp(wp.motor_centroid)
    burt.movel_pedestal([0.12,0.1635,0.019,np.sqrt(pi*pi/2),np.sqrt(pi*pi/2),0],robot=urnie)

    # insert motor into fixing plate
    burt.insert([-0.05,0,0],max_force=45,hunting_radius=0.001,rev=0.0003,circles=3,th=0.1,more=True)
    #if insert_motor(burt,axis=[-0.042,0,0],angle=30.0,max_force=40) == False:
    #    burt.translatel_rel([0.001,0,0])
    #    insert_motor(burt,axis=[-0.042,0,0],angle=-60.0,max_force=40)
    #    burt.translatel_rel([0.001,0,0])
    #    insert_motor(burt,axis=[-0.015,0,0],angle=-30.0,max_force=40)
    #else:
    #    burt.translatel_rel([0.001,0,0])
    #    insert_motor(burt,axis=[-0.015,0,0],angle=30,max_force=40)

    #burt.set_tcp(wp.pincher_tcp)

    while urnie_flag == True:
        time.sleep(0.1)

    # screw in all screws
    for i in range(0,6):
        if i>0:
            urnie.home()
            get_grease(urnie,'m3')
            get_screw(urnie,'m3',[0.03+0.03*int(i/3),0.015+0.03*(i%3)])
        motor_screw(urnie,[0.165+0.0155*np.sin((30.0+60.0*i)*pi/180.0),0.0355+0.0155*np.cos((30.0+60.0*i)*pi/180.0)],3)

    # stow allen key
    _thread.start_new_thread(stow_allen_key,(urnie,'m3'))

    burt.open_gripper(0)
    burt.open_gripper(1)
    burt.open_gripper(3,wait=False)
    burt.translatel_rel([0.05,0,0])
    burt.home()
    while urnie_flag == True:
        time.sleep(0.1)
    return

def subtask_b(burt,urnie,coords):
    """
    # Target Parts: small pulley, m2 set screw
    # Task: Assembling the motor-shaft-pulley to the motor shaft
    # Points: 3
    # Notes:
    """
    # init robots
    urnie.cal_fingers(wait=False)
    urnie.open_gripper(5,wait=False)

    # get small pulley
    urnie.movejl(urnie.pedestal.right_tray([0.03,0.245,0.03,0,pi,0]))
    urnie.wait_for_gripper()
    urnie.force_move([0,0,-0.05],force=35)
    urnie.close_gripper(1)
    urnie.close_gripper(1)
    urnie.translatel_rel([0,0,0.05])

    # move to pedestal
    demand_joints = copy.deepcopy(wp.urnie_pedestalj)
    demand_joints[5]-=pi/2.0
    urnie.movej(demand_joints)
    urnie.translatel_pedestal([-0.01,0.165,0.042])

    # insert
    urnie.insert([0.035,0,0],max_force=40,hunting_radius=0.0015,circles=3,rev=0.0005,th=0.05,more=True)
    n=0
    r=0.0005
    while urnie.getl()[0]<0.360 and n<7:
        if urnie.insert([0.01,0,0],max_force=40,hunting_radius=r,circles=1,rev=0,more=True) == False:
            r+=0.0005
        else:
            r=0.0005
        n+=1
    #urnie.cal_fingers()
    urnie.open_gripper(0)
    urnie.open_gripper(2)
    urnie.translatel_rel([-0.01,0,0])
    urnie.close_gripper(2)
    demand_pose = urnie.getl()
    demand_pose[0] = wp.ac_br[0]+0.02
    urnie.translatel(demand_pose)

    # release and get allen key
    urnie.open_gripper(0)
    urnie.open_gripper(2)
    urnie.translatel_rel([-0.05,0,0])
    get_allen_key(urnie,'m2',False)

    # move to pedestal
    urnie.movejl_pedestal([0.024,0.166,0.1,0,pi,0])

    # screw in
    urnie.insert([0,0,-0.05],max_force=35,hunting_radius=0.0005,circles=3,th=0.1,R=4)
    urnie.screw_in(3,R=10)

    # move away and stow allen key
    urnie.translatel_rel([0,0,0.02])
    stow_allen_key(urnie,'m2')
    return

def subtask_c(burt,urnie,coords):
    """
    # Target Parts: output shaft, washers, bearing housing, m4 screws
    # Task: Assembling the output shaft, screws to secure the output shaft, washers, double  bearings, and the screws to attach the bearings to the output shaft fixing plate
    # Points: 5
    # Notes:
    """
    subtask_c_bearing_housing(burt,urnie,coords)
    return subtask_c_output_shaft(burt,urnie,coords)

def subtask_c_bearing_housing(burt,urnie,coords):
    """
    # Target Parts: output shaft, washers, bearing housing, m4 screws
    # Task: Assembling the output shaft, screws to secure the output shaft, washers, double  bearings, and the screws to attach the bearings to the output shaft fixing plate
    # Points: 5
    # Notes: this fn assembles the bearing housing
    """
    # init robots
    urnie.cal_fingers(wait=False)
    #burt.open_gripper(0)
    #burt.open_gripper(4,wait=False)
    urnie.open_gripper(5)
    urnie.close_gripper(0)
    urnie.close_gripper(5,wait=False)
    burt.rotate_rel([pi/9.0,0,0,0,0,0],wait=False)
    urnie.home()

    # pick up bearing housing
    urnie.movejl(urnie.pedestal.left_tray([0.045,0.245,0.153,0,pi,0]))
    urnie.translatel_rel([0,0,-0.13])
    urnie.wait_for_gripper()
    urnie.insert(axis=[0,0,-0.03],max_force=30)
    urnie.open_gripper(2)
    urnie.translatel_rel([0,0,0.15])
    urnie.home()

    # move to pedestal
    demand_joints = copy.deepcopy(wp.urnie_pedestalj)
    demand_joints[5]-=pi/2.0
    urnie.movej(demand_joints)
    urnie.translatel_pedestal([-0.025,0.034,0.045])

    # insert bearing housing
    urnie.wait_for_gripper()
    urnie.insert([0.07,0,0],max_force=40,hunting_radius=0.001,circles=3,rev=0,th=0.05,more=True)
    urnie.open_gripper(3)
    n=0
    r=0.0005
    while urnie.getl()[0]<0.373 and n<15:
        if urnie.insert([0.025,0,0],max_force=40,hunting_radius=r,circles=1,rev=0.0005,more=True) == False:
            r+=0.0005
        else:
            r=0.0005
        n+=1
    urnie.insert([0.025,0,0],max_force=50,hunting_radius=0.0005,circles=3,rev=0)
        

    # move burt to housing
    #burt.set_tcp(wp.motor_centroid)
    #burt.movejl_pedestal([0.1,0.033,0.044,np.sqrt(pi*pi/2),np.sqrt(pi*pi/2),0],robot=urnie)
    #burt.force_move([-0.06,0,0],force=35)
    #burt.translatel_rel([0.003,0,0])
    #stored_pose = burt.getl()
    #time.sleep(0.1)
    #burt.force_move([0,0.05,0],force=35)
    #y1 = burt.getl()[1]
    #burt.translatel_rel([0,-0.005,0])
    #time.sleep(0.1)
    #burt.force_move([0,-0.05,0],force=35)
    #y2 = burt.getl()[1]
    #stored_pose[1] = (y1+y2)/2.0
    #burt.translatel(stored_pose)
    #burt.close_gripper(0)
    #burt.close_gripper(3)
    #burt.close_gripper(3)
    #urnie.translatel_rel([-0.05,0,0])
    #burt.open_gripper(0)
    #burt.open_gripper(1,wait=False)

    urnie.movel_tool([0,0,0,0,-pi/18.0,0])
    urnie.translatel_rel([0,0,0.001])
    urnie.translatel_rel([-0.05,0,0])

    # get allen key
    get_allen_key(urnie,'m4_short',True)

    # align hole
    #urnie.movej(wp.urnie_pedestalj)
    #urnie.translatel_pedestal([-0.025,0.036,0.065])
    #insert_housing(burt,urnie)
    #insert_housing(burt,urnie,angle=10)
    #urnie.translatel_rel([-0.05,0,0])

    for i in range(0,4):
        if i==2:
            get_grease(urnie,'m4_short')
        get_screw(urnie,'m4_short',[0.13+0.03*int(i/2),0.045+0.03*(i%2)])
        motor_screw(urnie,[0.035+0.022*np.sin(90.0*i*pi/180.0),0.0425+0.022*np.cos(90.0*i*pi/180.0)],4)

    # stow allen key and home robots
    urnie.home()
    stow_allen_key(urnie,'m4_short')
    #global urnie_flag
    #_thread.start_new_thread(stow_allen_key,(urnie,'m4_short'))
    #burt.open_gripper(0)
    #burt.open_gripper(1)
    #burt.open_gripper(4,wait=False)
    #burt.translatel_rel([0.05,0,0])
    #burt.home()
    #while urnie_flag == True:
    #    time.sleep(0.1)
    return

def subtask_c_output_shaft(burt,urnie,coords):
    """
    # Target Parts: output shaft, washers, bearing housing, m4 screws
    # Task: Assembling the output shaft, screws to secure the output shaft, washers, double  bearings, and the screws to attach the bearings to the output shaft fixing plate
    # Points: 5
    # Notes: this fn assembles the output shaft
    """
    # init robots
    urnie.cal_fingers(wait=False)
    urnie.open_gripper(0)
    urnie.open_gripper(5,wait=False)
    burt.to_switch(wait=False)

    # get shaft with urnie
    urnie.movejl(urnie.pedestal.right_tray([0.03,0.14,0.095,0,pi,0]))
    urnie.wait_for_gripper()
    urnie.translatel_rel([0,0,-0.05])
    urnie.close_gripper(3)
    urnie.close_gripper(3)
    urnie.translatel_rel([0,0,0.03])
    urnie.home()
    urnie.translatel_rel([-0.1,0,0],wait=False)

    # get endcap with burt
    burt.movejl(burt.urnie_to_burt(urnie.pedestal.right_tray([0.09,0.175,0.08,0,pi,0])))
    burt.translatel_rel([0,0,-0.05])
    burt.wait_for_gripper()
    burt.force_move([0,0,-0.05],force=35)
    burt.translatel_rel([0,0,0.005])
    burt.close_gripper(2)
    burt.close_gripper(1)
    burt.close_gripper(1)
    burt.close_gripper(1)
    burt.translatel_rel([0,0,0.08])
    burt.home()

    # move urnie to pedestal
    demand_joints = copy.deepcopy(wp.urnie_pedestalj)
    demand_joints[5]-=pi/2.0
    urnie.movej(demand_joints,wait=False)
    burt.movej(wp.burt_pedestalj,wait=False)
    urnie.translatel_pedestal([-0.05,0.035,0.042],wait=False)
    burt.translatejl_pedestal([0.08,0.0335,0.044],robot=urnie)

    # insert shaft into bearing housing
    urnie.insert([0.05,0,0],max_force=40,hunting_radius=0.002,circles=3,rev=0,th=0.05,more=True)
    n=0
    r=0.001
    while urnie.getl()[0]<0.340 and n<8:
        urnie.translatel_rel([-0.001,0,0])
        if urnie.insert([0.015,0,0],max_force=50,hunting_radius=r,circles=1,rev=0,more=True) == False:
            r+=0.0005
        else:
            r=0.001
        n+=1

    # insert endcap
    burt.insert([-0.03,0,0],max_force=27,hunting_radius=0.0015,more=True)
    burt.open_gripper(0)
    burt.open_gripper(0)
    burt.open_gripper(0)
    burt.open_gripper(0)
    burt.open_gripper(0)
    burt.open_gripper(1)
    burt.open_gripper(1)
    burt.to_switch(wait=False)
    burt.translatel_rel([0.03,0,0])
    stored_pose = burt.getl()

    # get screw with burt
    burt.home()
    burt.movejl(burt.urnie_to_burt(urnie.pedestal.right_tray([0.163,0.0135,0.08,0,pi,0])))
    burt.translatel_rel([0,0,-0.07])
    burt.force_move([0,0,-0.03])
    burt.translatel_rel([0,0,0.008])
    burt.close_gripper(3)
    burt.close_gripper(1)
    burt.close_gripper(1)
    burt.close_gripper(1)
    burt.translatel_rel([0,0,0.1])
    burt.home()

    # screw in
    burt.movejl(stored_pose)
    #burt.insert([-0.033,0,0],max_force=27.5)
    burt.force_move([-0.033,0,0],force=27.5)
    urnie.screw_in(4,R=20)
    urnie.open_gripper(0)
    urnie.open_gripper(1)
    urnie.open_gripper(2,wait=False)
    urnie.translatel_rel([-0.05,0,0])
    urnie_stored_pose = urnie.getl()
    urnie.home()

    # get spacer with urnie
    urnie.movejl(urnie.pedestal.right_tray([0.16,0.175,0.02,0,pi,0]))
    urnie.force_move([0,0,-0.05],force=35)
    urnie.translatel_rel([0,0,0.002])
    urnie.close_gripper(2)
    urnie.close_gripper(2)
    urnie.translatel_rel([0,0,0.08])

    # insert spacer
    urnie.movej(wp.urnie_pedestalj)
    urnie.translatejl(stored_pose)
    urnie.insert([0.05,0,0],max_force=40,more=True)
    urnie.open_gripper(0)
    urnie.open_gripper(1)
    urnie.translatel_rel([-0.05,0,0],wait=False)
    burt.force_move([-0.05,0,0],force=50)

    # home
    urnie.home()
    print(urnie_stored_pose)

    # reset burt when testing
    burt.open_gripper(0)
    burt.open_gripper(0)
    burt.open_gripper(0)
    burt.open_gripper(1)
    burt.to_switch(wait=False)
    burt.translatel_rel([0.03,0,0])
    burt_stored_pose = burt.getl()
    print(burt_stored_pose)
    burt.home()
    return urnie_stored_pose, burt_stored_pose

def subtask_d(burt,urnie,coords,urnie_stored_pose=[0.313453, -0.403492, 0.114494, -1.2132, 1.20591, -1.21221],burt_stored_pose=[0,0,0,0,0,0]):
    """
    # Target Parts: large pulley, m4 screws
    # Task: Assembling the output-shaft-pulley to the output shaft
    # Points: 3
    # Notes: burt starts pushing against the shaft
    """
    # init robots
    urnie.cal_fingers(wait=False)
    urnie.open_gripper(5,wait=False)
    burt.to_switch(wait=False)
    burt.rotate_rel([pi/9.0,0,0,0,0,0])

    # get output pulley
    urnie.movejl(urnie.pedestal.left_tray([0.145,0.245,0.02,0,pi,0]))
    urnie.wait_for_gripper()
    urnie.force_move([0,0,-0.05],force=40)
    urnie.translatel_rel([0,0,0.002])
    urnie.close_gripper(2)
    urnie.close_gripper(2)
    urnie.translatel_rel([0,0,0.05])
    urnie.home()

    # move to pedestal
    demand_joints = copy.deepcopy(wp.urnie_pedestalj)
    demand_joints[5]+=pi/2.0
    burt.movej(wp.burt_pedestalj,wait=False)
    urnie.movej(demand_joints,wait=False)
    burt.translatejl_pedestal([0.08,0.0335,0.044],robot=urnie)
    urnie.translatel(urnie_stored_pose,wait=False)
    burt.force_move([-0.04,0,0],force=40)
    burt.translatel_rel([0.005,0,0])
    burt.close_gripper(2)
    burt.close_gripper(2)
    burt.force_move([-0.01,0,0],force=30)

    # insert pulley
    urnie.force_move([0.05,0,0],force=30)
    urnie.open_gripper(1)    
    urnie.close_gripper(0)   
    urnie.close_gripper(0)   
    urnie.close_gripper(0)   
    urnie.close_gripper(0)
    urnie.insert([0.04,0,0],max_force=40,hunting_radius=0.002,more=True)
    n=0
    r=0.001
    while urnie.getl()[0]<0.355 and n<5:
        urnie.translatel_rel([-0.001,0,0])
        if urnie.insert([0.015,0,0],max_force=50,hunting_radius=r,circles=1,rev=0,more=True) == False:
            r+=0.0005
        else:
            r=0.001
        n+=1
    urnie.force_move([0.1,0,0],force=50)
    urnie.cal_fingers()

    # get allen key
    burt.open_gripper(0)
    burt.open_gripper(1)
    burt.to_switch(wait=False)
    urnie.open_gripper(0)
    urnie.open_gripper(1)
    urnie.open_gripper(3,wait=False)
    urnie.translatel_rel([-0.06,0,0])
    get_allen_key(urnie,'m4_long',False)

    # move burt to pulley
    burt.translatel_rel([0.03,0,0])
    burt.home()
    burt.movej(wp.burt_pulleyj)
    burt.translatel_rel([0,0,-0.065])
    burt.wait_for_gripper()
    burt.force_move([0,0.05,0],force=60)

    # screw in
    urnie.movejl_pedestal([0.013,0.028,0.155,0,pi,0])
    urnie.insert([0,0,-0.02],max_force=35,hunting_radius=0.0015,R=4)
    urnie.screw_in(0,R=8)
    urnie.translatel_rel([0,0,0.02])
    urnie.translatel_pedestal([0.013,0.044,0.155])
    urnie.insert([0,0,-0.02],max_force=35,hunting_radius=0.0015,R=4)
    urnie.screw_in(0,R=8)
    urnie.translatel_rel([0,0,0.02])

    # move away
    urnie.home()
    burt.translatel_rel([0,-0.05,0])
    burt.home()

    # stow allen key
    stow_allen_key(urnie,'m4_long')    
    return

def subtask_e(burt,urnie,coords):
    """
    # Target Parts: 
    # Task: Assembling the tension pulley to the output shaft fixing plate and adjusting the belt tension 
    # Points: 5
    # Notes:
    """
    # init robots
    burt.to_switch(wait=False)

    # get allen key with urnie
    get_allen_key(urnie,'m4_short',True)

    # get retainer pin with burt
    burt.movejl(burt.urnie_to_burt(urnie.pedestal.right_tray([0.095,0.11,0.08,0,pi,0])))
    burt.translatel_rel([0,0,-0.04])
    burt.wait_for_gripper()
    burt.force_move([0,0,-0.05],force=40)
    burt.translatel_rel([0,0,0.005])
    burt.close_gripper(4)
    burt.force_move([0,0,-0.01],force=50)
    burt.close_gripper(1)
    burt.translatel_rel([0,0,0.1])
    burt.home()
    
    # pass retainer pin to urnie
    burt.movej(wp.burt_passj,wait=False)
    urnie.movej(wp.urnie_passj)
    burt.translatel_rel([0,0,0.03])
    burt_pose = burt.getl()
    burt_pose[0]-=0.05
    urnie_pose = urnie.getl()
    urnie.move_ors([burt_pose[0],burt_pose[1],burt_pose[2],urnie_pose[3],urnie_pose[4],urnie_pose[5]])
    urnie.insert([0.03,0,0],hunting_radius=0.0005,R=6,more=True)
    burt.open_gripper(0)
    burt.open_gripper(0)
    burt.open_gripper(1)
    burt.to_switch()
    burt.open_gripper(2,wait=False)
    urnie.translatel_rel([-0.045,0,0])
    burt.home(wait=False)
    urnie.movej(wp.urnie_passj)
    urnie.translatel_rel([-0.05,0.1,0])

    # get small pulley with burt
    burt.movejl(burt.urnie_to_burt(urnie.pedestal.left_tray([0.035,0.03,0.01,0,pi,0])))
    burt.wait_for_gripper()
    burt.force_move([0,0,-0.05],force=40)
    burt.translatel_rel([0,0,0.01])
    burt.close_gripper(2)
    burt.close_gripper(1)
    burt.translatel_rel([0,0,0.1])
    burt.home()

    # put small pulley on retainer pin
    burt.movej(wp.burt_passj)
    urnie_pose = urnie.getl()
    urnie_pose[0]+=0.1
    burt_pose = burt.getl()
    burt.move_ors([urnie_pose[0],urnie_pose[1],urnie_pose[2],burt_pose[3],burt_pose[4],burt_pose[5]])
    burt.insert(axis=[-0.05,0,0],max_force=35,hunting_radius=0.002,more=True)
    burt.open_gripper(0)
    burt.open_gripper(1)
    burt.open_gripper(1,wait=False)
    burt.translatel_rel([0.05,0,0])
    burt.to_switch(wait=False)
    burt.home()

    # get spacer with burt
    burt.movejl(burt.urnie_to_burt(urnie.pedestal.right_tray([0.16,0.11,0.1,0,pi,0])))
    burt.translatel_rel([0,0,-0.09])
    burt.wait_for_gripper()
    burt.force_move([0,0,-0.05],force=40)
    burt.translatel_rel([0,0,0.005])
    burt.close_gripper(3)
    burt.close_gripper(1)
    burt.translatel_rel([0,0,0.1])
    burt.home()

    # put spacer on retainer pin
    burt.movej(wp.burt_passj)
    burt_pose = burt.getl()
    burt.move_ors([urnie_pose[0],urnie_pose[1],urnie_pose[2],burt_pose[3],burt_pose[4],burt_pose[5]])
    burt.insert(axis=[-0.05,0,0],max_force=35,hunting_radius=0.002,more=True)
    burt.open_gripper(0)
    burt.open_gripper(1)
    burt.to_switch(wait=False)
    burt.translatel_rel([0.05,0,0])
    burt.home()

    # get washer with burt
    burt.movejl(burt.urnie_to_burt(urnie.pedestal.right_tray([0.16,0.23,0.1,0,pi,0])))
    burt.translatel_rel([0,0,-0.09])
    burt.wait_for_gripper()
    burt.force_move([0,0,-0.05],force=40)
    burt.translatel_rel([0,0,0.0005])
    burt.close_gripper(3)
    burt.close_gripper(1)
    burt.translatel_rel([0,0,0.1])
    burt.home()

    # put washer on retainer pin
    burt.movej(wp.burt_passj)
    burt_pose = burt.getl()
    burt.move_ors([urnie_pose[0],urnie_pose[1],urnie_pose[2],burt_pose[3],burt_pose[4],burt_pose[5]])
    burt.insert(axis=[-0.05,0,0],max_force=35,hunting_radius=0.002,more=True)
    burt.open_gripper(0)
    burt.open_gripper(1)
    burt.to_switch(wait=False)
    burt.translatel_rel([0.05,0,0])
    burt.home()

    # get washer with burt
    burt.movejl(burt.urnie_to_burt(urnie.pedestal.right_tray([0.16,0.26,0.01,0,pi,0])))
    burt.wait_for_gripper()
    burt.force_move([0,0,-0.05],force=40)
    burt.translatel_rel([0,0,0.0005])
    burt.close_gripper(3)
    burt.close_gripper(1)
    burt.translatel_rel([0,0,0.1])
    burt.home()

    # move urnie to pedestal
    urnie.movej(wp.urnie_pedestalj)
    urnie.translatel_pedestal([-0.02,0.095,0.08])

    # insert tensioner
    urnie.insert(axis=[0.05,0,0],max_force=35,hunting_radius=0.002,more=True)
    tcp = copy.deepcopy(wp.rotary_tcp)
    tcp[2]+=0.03
    urnie.set_tcp(tcp)
    urnie.movel_rel([0,0,0,pi/36.0,0,0])
    urnie.movel_rel([0,0,0,-pi/36.0,0,0])

    # put washer on assembly
    burt.movej(wp.burt_pedestalj)
    burt.translatel_pedestal([0.06,0.095,0.08],robot=urnie)
    burt.insert(axis=[-0.03,0,0],max_force=30,hunting_radius=0.002,more=True)
    stored_pose = burt.getl()
    burt.open_gripper(0)
    burt.open_gripper(1)
    burt.to_switch(wait=False)
    burt.translatel_rel([0.05,0,0])
    burt.home()

    # get nut with burt
    burt.movejl(burt.urnie_to_burt(urnie.pedestal.right_tray([0.16,0.17,0.01,0,pi,0])))
    burt.wait_for_gripper()
    burt.force_move([0,0,-0.05],force=40)
    burt.translatel_rel([0,0,0.002])
    burt.close_gripper(3)
    burt.close_gripper(1)
    burt.translatel_rel([0,0,0.1])
    burt.home()

    # move nut to tensioner
    burt.movej(wp.urnie_pedestalj)
    stored_pose[0]+=0.02
    burt.translatel(stored_pose)
    burt.force_move([-0.03,0,0],force=40)

    # screw in
    urnie.screw_in(4,R=5,robot=burt)

    # tension belt
    urnie_pose = urnie.getl()
    urnie.translatel_rel([0,0,-0.03],vel=0.01,wait=False)
    urnie_pose[2]-=0.03
    burt.follow(urnie_pose)
    urnie.screw_in(4,R=10,robot=burt)

    # release and home
    urnie.translatel_rel([-0.03,0,0])
    global urnie_flag
    _thread.start_new_thread(stow_allen_key,(urnie,'m4_short'))
    burt.open_gripper(0)
    burt.open_gripper(1)
    burt.to_switch(wait=False)
    burt.translatel_rel([0.03,0,0])
    burt.home()
    while urnie_flag == True:
        time.sleep(0.1)
    return

def subtask_f(burt,urnie,coords):
    """
    # Target Parts: motor fixing plate, m4 screws
    # Task: Assembling the motor fixing plate and the base plate with the screws to connect both 
    # Points: 3
    # Notes:
    """
    # get allen key
    global urnie_flag
    _thread.start_new_thread(get_allen_key,(urnie,'m4_long',True))

    # set burt gripper
    burt.cal_gripper(wait=False)
    burt.close_gripper(1,wait=False)

    # home burt
    burt.home(wait=False)

    # move to fixing plate
    demand_joints = burt.get_inverse_kin(burt.calc_pedestal([0.19,-0.095,0.035,0,pi,0],robot=urnie))
    demand_joints[5]-=pi/2.0
    burt.movej(demand_joints)

    # pick up fixing plate
    burt.translatel_rel([0,0,-0.07])
    burt.close_gripper(3)
    burt.close_gripper(3)
    burt.translatel_rel([0,0,0.15])

    # get screw
    while urnie_flag == True:
        time.sleep(0.1)
    _thread.start_new_thread(get_screw,(urnie,'m4_long',[0.1,0.015]))

    # move to pedestal
    burt.movel_pedestal([0.0395,0.147,0.115,0,pi,0],robot=urnie)
    burt.force_move([0,0,-0.1])
    burt.open_gripper(0)
    burt.open_gripper(1)
    burt.movel_tool([0,0,0,-pi/3.0,0,0])
    burt.close_gripper(1)
    burt.close_gripper(1)

    # wait for urnie
    while urnie_flag == True:
        time.sleep(0.1)

    # move urnie to first hole
    urnie.translatejl_pedestal([0.051,0.194,0.2])
    urnie.translatel_rel([0,0,-0.06])
    
    # screw in
    stored_pose = urnie.getl()
    urnie.insert(axis=[0,0,-0.035],max_force=30,hunting_radius=0.003,circles=3)
    demand_pose = urnie.getl()
    demand_pose[0]=stored_pose[0]
    demand_pose[1]=stored_pose[1]
    demand_pose[2]+=0.001
    urnie.translatejl(demand_pose)
    urnie.screw_in(4,R=20)
    urnie.translatel_rel([0,0,0.1])

    # get second screw
    urnie.home()
    _thread.start_new_thread(get_screw,(urnie,'m4_long',[0.1,0.045]))
    burt.open_gripper(1)
    burt.open_gripper(1)
    burt.open_gripper(2,wait=False)
    burt.translatel_rel([0,0,0.05])
    burt.home()

    # move urnie to second hole
    while urnie_flag == True:
        time.sleep(0.1)
    urnie.translatejl_pedestal([0.051,0.136,0.2])
    urnie.translatel_rel([0,0,-0.06])

    # screw in
    stored_pose = urnie.getl()
    urnie.insert(axis=[0,0,-0.035],max_force=30,hunting_radius=0.003,circles=3)
    demand_pose = urnie.getl()
    demand_pose[0]=stored_pose[0]
    demand_pose[1]=stored_pose[1]
    demand_pose[2]+=0.001
    urnie.translatejl(demand_pose)
    urnie.screw_in(4,R=20)
    urnie.translatel_rel([0,0,0.1])

    # stow allen key
    stow_allen_key(urnie,'m4_long')
    return

def subtask_g(burt,urnie,coords,get_key=True):
    """
    # Target Parts: output shaft fixing plate, screws
    # Task: Assembling the output shaft fixing plate and the base plate with the screws to connect both 
    # Points: 3
    # Notes:
    """
    # get allen key
    global urnie_flag
    if get_key == True:
        _thread.start_new_thread(get_allen_key,(urnie,'m4_long',True))

    # set burt gripper
    burt.cal_gripper(wait=False)
    burt.close_gripper(1,wait=False)

    # home burt
    burt.home(wait=False)

    # move to fixing plate
    demand_joints = burt.get_inverse_kin(burt.calc_pedestal([0.152,-0.041,0.055,0,pi,0],robot=urnie))
    demand_joints[5]-=pi/2.0
    burt.movej(demand_joints)

    # pick up fixing plate
    burt.translatel_rel([0,0,-0.07])
    burt.close_gripper(3)
    burt.close_gripper(3)
    burt.translatel_rel([0,0,0.15])

    # get screw
    while urnie_flag == True:
        time.sleep(0.1)
    _thread.start_new_thread(get_screw,(urnie,'m4_long',[0.1,0.075]))

    # move to pedestal
    burt.movel_pedestal([0.0415,0.0565,0.135,0,pi,0],robot=urnie)
    burt.force_move([0,0,-0.1])
    burt.open_gripper(0)
    burt.open_gripper(1)
    burt.movel_tool([0,0,0,-pi/3.0,0,0])
    burt.close_gripper(1)
    burt.close_gripper(1)

    # move urnie to first hole
    while urnie_flag == True:
        time.sleep(0.1)
    urnie.home()
    urnie.translatejl_pedestal([0.053,0.110,0.23])
    urnie.translatel_rel([0,0,-0.09])
    
    # screw in
    stored_pose = urnie.getl()
    urnie.insert(axis=[0,0,-0.035],max_force=35,hunting_radius=0.003,circles=3)
    demand_pose = urnie.getl()
    demand_pose[0]=stored_pose[0]
    demand_pose[1]=stored_pose[1]
    demand_pose[2]+=0.001
    urnie.translatejl(demand_pose)
    urnie.screw_in(4,R=20)
    urnie.translatel_rel([0,0,0.1])

    # get second screw
    urnie.home()
    _thread.start_new_thread(get_screw,(urnie,'m4_long',[0.13,0.015]))
    burt.open_gripper(0)
    burt.open_gripper(1)
    burt.open_gripper(3,wait=False)
    burt.translatel_rel([0,0,0.08])
    burt.home()

    # move urnie to second hole
    urnie.translatejl_pedestal([0.053,0.006,0.23])
    urnie.translatel_rel([0,0,-0.09])

    # screw in
    stored_pose = urnie.getl()
    urnie.insert(axis=[0,0,-0.035],max_force=35,hunting_radius=0.003,circles=3)
    demand_pose = urnie.getl()
    demand_pose[0]=stored_pose[0]
    demand_pose[1]=stored_pose[1]
    demand_pose[2]+=0.001
    urnie.translatejl(demand_pose)
    urnie.screw_in(4,R=20)
    urnie.translatel_rel([0,0,0.1])

    # stow allen key
    stow_allen_key(urnie,'m4_long')
    return

def subtask_h(burt,urnie,coords):
    """
    # Target Parts: 
    # Task: Assembling the belt
    # Points: 5
    # Notes:
    """
    # init robots
    burt.to_switch(wait=False)

    # get belt with burt
    burt.movejl(burt.urnie_to_burt(urnie.pedestal.left_tray([0.155,0.14,0.01,0,pi,0])))
    burt.wait_for_gripper()
    burt.force_move([0,0,-0.05],force=35)
    burt.translatel_rel([0,0,0.001])
    burt.close_gripper(3)
    burt.close_gripper(1)
    burt.close_gripper(1)
    burt.translatel_rel([0,0,0.05])
    burt.home()

    # move to pedestal
    burt.movej(wp.burt_beltj)
    burt.translatel_rel([-0.03,0,-0.094])

    # hook over large pulley
    current_pose = burt.getl()
    demand_pose = burt.calc_pedestal([0.028,0.037,0.1,0,pi,0],robot=urnie)
    burt.movel([demand_pose[0],demand_pose[1],current_pose[2],0,1.6643,0])
    
    # rotate over to small pulley
    current_pose = burt.getl()
    burt.movel([current_pose[0]-0.015,current_pose[1]+0.105,current_pose[2]-0.03,1.5727,-3.8024,1.7238])
    current_pose = burt.getl()
    demand_z = burt.calc_pedestal([0.005,0.15,0.044,0,pi,0],robot=urnie)[2]
    burt.movel([current_pose[0],current_pose[1]+0.02,demand_z,3.0960,-2.2457,2.1483])
    burt.force_move([0.02,0,0],force=40)
    burt.translatel_rel([-0.001,0,0])
    time.sleep(0.5)
    burt.force_move([0,0.045,0],force=40)

    # hook over small pulley
    burt.movel_tool([0,0,0,0,pi/9.0+pi/36.0,0])
    burt.movel_tool([-0.003,0,0.006,0,0,0])
    burt.movel_tool([0,0,0,0,pi/9.0,0])
    burt.open_gripper(1)
    burt.open_gripper(1)
    urnie.rotate_rel([-pi/12.0,0,0,0,0,0],wait=False)
    burt.translatel_rel([-0.01,0,0])
    burt.to_switch()
    burt.translatel_rel([0,0,0.05])

    # home
    burt.home()
    urnie.home()
    return

def tension_belt(burt,urnie):
    """
    # subtask of subtask_e()
    """

    return

def get_screw(urnie,key,coords):
    """
    # get a screw from tray
    """
    global urnie_flag
    urnie_flag=True
    urnie.home()
    z = 0.1
    r=6
    if key == 'm4_long':
        z = 0.11
        coords[0]+=0.001
    elif key == 'm4_short':
        z = 0.06
        coords[0]+=0.0005
    elif key == 'm3':
        z = 0.070
        r=7
        coords[0]+=0.001
    urnie.movejl(urnie.pedestal.right_tray([coords[0],coords[1],z,0,pi,0]))
    urnie.insert([0,0,-0.05],max_force=35,hunting_radius=0.0005,R=r)
    urnie.translatel_rel([0,0,0.05],acc=0.1,vel=0.1)
    urnie.home()
    urnie_flag=False
    return

def get_grease(urnie,key):
    z = 0.1
    if key == 'm4_long':
        z = 0.1
    if key == 'm4_short':
        z = 0.05
    if key == 'm3':
        z = 0.06
    urnie.movejl_pedestal([-0.02+random.randrange(-15,15,1)/1000.0,-0.07+random.randrange(-15,15,1)/1000.0,z,0,pi,0])
    urnie.force_move([0,0,-0.1],force=40)
    urnie.translatel_rel([0,0,0.003])
    urnie.rotate_rel([0,0,0,0,0,pi/2.0],vel=1.0)
    urnie.translatel_rel([0,0,0.05])
    urnie.home(vel=1.0)
    return

def motor_screw(urnie,coords,M):
    """
    # subtask of subtask_a()
    """
    # move to pedestal
    urnie.cal_fingers(wait=False)
    urnie.movej(wp.urnie_pedestalj)
    urnie.translatel_pedestal([-0.03,coords[0],coords[1]])

    # screw in
    stored_pose = urnie.getl()
    urnie.insert(axis=[0.03,0,0],max_force=40,hunting_radius=0.002,circles=2,rev=0.0003,th=0.05,more=True)
    demand_pose = urnie.getl()
    demand_pose[0]-=0.0005
    demand_pose[1]=stored_pose[1]
    demand_pose[2]=stored_pose[2]
    urnie.translatejl(demand_pose)
    urnie.screw_in(M,R=20)
    urnie.translatel_rel([-0.05,0,0])
    return

def insert_motor(burt,axis=[-0.05,0,0],angle=45.0,max_force=35):
    """
    probe for motor hole by rotating around motor centroid 
    """
    iter = 45
    travel_thresh = abs(axis[0])-0.003
    burt.set_tcp(wp.motor_centroid)
    start_pose = burt.getl()
    burt.force_move(axis,acc=0.1,vel=0.01,force=max_force)
    end_pose = burt.getl()
    dif = end_pose[0]-start_pose[0]
    axis[0] -= dif
    #print(dif)
    if abs(dif)<travel_thresh:
        #have hit the fixing plate so hunt
        burt.movel_tool([-0.0005,0,0,0,0,0])
        time.sleep(0.1)
        start_force = burt.get_forces()[0]
        force = start_force
        print("hunting start force: ",start_force)
        for i in range(0,iter):
            burt.movel_tool([0,0,0,0,-(angle*pi)/(iter*180.0),0],acc=0.1)
            burt.translatel([end_pose[0]-0.0005,end_pose[1],end_pose[2]],acc=0.1)
            time.sleep(0.1)
            force = burt.get_forces()[0]
            print("measured force: ",force)
            if abs(force)<abs(0.2*start_force):
                burt.force_move(axis,acc=0.1,vel=0.01,force=max_force)
                
                burt.set_tcp(wp.pincher_tcp)
                return True
        burt.set_tcp(wp.pincher_tcp)
        return False
    burt.set_tcp(wp.pincher_tcp)
    return True

def insert_housing(burt,urnie,angle=20):
    """
    probe for motor hole by rotating around motor centroid 
    """
    burt.set_tcp(wp.motor_centroid)

    urnie.force_move([0.04,0,0],force=40)
    urnie.translatel_rel([-0.0005,0,0])
    n = 0
    time.sleep(0.2)
    start_force = urnie.get_forces()[0]
    print("start_force: ",start_force)
    force = start_force
    while(force<0.1*start_force):
        force = urnie.get_forces()[0]
        print(force)
        if n<angle/2:
            burt.movel_tool([0,0,0,0,-pi/180.0,0],acc=0.1)
        elif n<3*angle/2:
            burt.movel_tool([0,0,0,0,pi/180.0,0],acc=0.1)
        elif n<3*angle:
            burt.movel_tool([0,0,0,0,-pi/180.0,0],acc=0.1)
        elif n<5*angle:
            burt.movel_tool([0,0,0,0,pi/180.0,0],acc=0.1)
        else:
            burt.set_tcp(wp.pincher_tcp)
            return False
        n+=1

    urnie.force_move([0.01,0,0],force=40)
    urnie.translatel_rel([-0.001,0,0])
    burt.set_tcp(wp.pincher_tcp)
    return True