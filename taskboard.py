import time
import copy
from math import pi
import math
import numpy as np
import scipy
import _thread

import waypoints as wp
import dual_robot_cal as drc

# thread flags
urnie_flag = False
burt_flag = False

class taskboard():
    def __init__(self):
        self.offset = np.array([wp.tc_tr[0],wp.tc_tr[1],math.asin((wp.tc_br[0]-wp.tc_tr[0])/(wp.tc_br[1]-wp.tc_tr[1]))])
        return

    def calc_pose(self,taskboard_pose):
        self.target_pose = taskboard_pose
        x = self.offset[0] + self.target_pose[0]*np.cos(self.offset[2]) + self.target_pose[1]*np.sin(self.offset[2])
        y = self.offset[1] - self.target_pose[0]*np.sin(self.offset[2]) + self.target_pose[1]*np.cos(self.offset[2])

        dz_x = (wp.tc_tl[2]-wp.tc_tr[2])/math.sqrt((wp.tc_tl[0]-wp.tc_tr[0])*(wp.tc_tl[0]-wp.tc_tr[0])+(wp.tc_tl[1]-wp.tc_tr[1])*(wp.tc_tl[1]-wp.tc_tr[1]))
        dz_y = (wp.tc_br[2]-wp.tc_tr[2])/math.sqrt((wp.tc_br[0]-wp.tc_tr[0])*(wp.tc_br[0]-wp.tc_tr[0])+(wp.tc_br[1]-wp.tc_tr[1])*(wp.tc_br[1]-wp.tc_tr[1]))
        z = taskboard_pose[2] + wp.tc_tr[2] + taskboard_pose[0]*dz_x + taskboard_pose[1]*dz_y
        return [x,y,z,taskboard_pose[3],taskboard_pose[4],taskboard_pose[5]]

def calibrate(urnie):
    urnie.set_tcp(wp.cal_rotary_tcp)

    demand_pose = copy.deepcopy(wp.tc_tl)
    demand_pose[2]+=0.01
    urnie.movejl(demand_pose)
    tc_tl1 = drc.keyboard_control(urnie)
    urnie.home()

    demand_pose = copy.deepcopy(wp.tc_tr)
    demand_pose[2]+=0.01
    urnie.movejl(demand_pose)
    tc_tr1 = drc.keyboard_control(urnie)
    urnie.home()

    demand_pose = copy.deepcopy(wp.tc_br)
    demand_pose[2]+=0.01
    urnie.movejl(demand_pose)
    tc_br1 = drc.keyboard_control(urnie)
    urnie.home()
    
    print("tc_tl =",tc_tl1)
    print("tc_tr =",tc_tr1)
    print("tc_br =",tc_br1)
    
    urnie.set_tcp(wp.rotary_tcp)
    return

def cal_allen_keys(urnie):
    urnie.set_tcp(wp.cal_rotary_tcp)
    
    demand_pose = copy.deepcopy(wp.allen_m6)
    demand_pose[2]+=0.01
    urnie.movejl(demand_pose)
    m6 = drc.keyboard_control(urnie)
    urnie.home()

    demand_pose = copy.deepcopy(wp.allen_m4)
    demand_pose[2]+=0.01
    urnie.movejl(demand_pose)
    m4 = drc.keyboard_control(urnie)
    urnie.home()

    demand_pose = copy.deepcopy(wp.allen_m3)
    demand_pose[2]+=0.01
    urnie.movejl(demand_pose)
    m3 = drc.keyboard_control(urnie)
    urnie.home()

    demand_pose = copy.deepcopy(wp.allen_m2)
    demand_pose[2]+=0.01
    urnie.movejl(demand_pose)
    m2 = drc.keyboard_control(urnie)
    urnie.home()
    
    print("allen_m6 =",m6)
    print("allen_m4 =",m4)
    print("allen_m3 =",m3)
    print("allen_m2 =",m2)
    
    urnie.set_tcp(wp.rotary_tcp)
    return

def get_allen_key(urnie,key):
    global urnie_flag
    urnie_flag=True
    # open and orient fingers
    urnie.cal_gripper(wait=False)

    # home
    urnie.home()

    # move above allen key
    if key == 'm6':
        urnie.movejl(wp.allen_m6)
    elif key == 'm4':
        urnie.movejl(wp.allen_m4)
    elif key == 'm3':
        urnie.movejl(wp.allen_m3)
    elif key == 'm2':
        urnie.movejl(wp.allen_m2)

    # pick up key
    urnie.wait_for_gripper()
    urnie.translatel_rel([0,0,-0.01])
    urnie.close_gripper(5)

    # move up and set key
    urnie.translatel_rel([0,0,0.06])
    urnie.translatel_rel([0,0.01,0])
    urnie.force_move([0, 0, -0.1],force=50)
    urnie.close_gripper(1,wait=False)
    urnie.translatel_rel([0,0,0.01])

    # home
    urnie.home()
    urnie_flag=False
    return

def stow_allen_key(urnie,key):
    urnie_flag=True
    force = 50
    # home
    urnie.home()

    # move above allen key hole
    if key == 'm6':
        demand_pose = copy.deepcopy(wp.allen_m6)
    elif key == 'm4':
        demand_pose = copy.deepcopy(wp.allen_m4)
    elif key == 'm3':
        demand_pose = copy.deepcopy(wp.allen_m3)
        force = 40
    elif key == 'm2':
        demand_pose = copy.deepcopy(wp.allen_m2)
        force = 30
    demand_pose[2]+=0.04
    urnie.movejl(demand_pose)

    # insert into hole
    urnie.insert(max_force=force)
    urnie.open_gripper(1)
    urnie.open_gripper(5,wait=False)
    urnie.translatel_rel([0,0,0.03])

    # home
    urnie.home()
    urnie_flag=False
    return

def part_1(burt,urnie,coords): # coords: [x,y]
    """
    # Target Parts: Bearings with Housing
    # Task: Insersion into a hole
    # Points: 4
    # Notes: 2 hands to turn over part
    """
    # open and orient fingers
    urnie.open_gripper(5,wait=False)
    urnie.close_gripper(5,wait=False)
    burt.cal_gripper(wait=False)

    # home robots
    urnie.home(wait=False)
    burt.home(wait=False)

    # pick part with burt
    demand_joints = burt.get_inverse_kin([coords[0],coords[1],0.05,0,pi,0])
    demand_joints[5] += pi/2
    burt.movej(demand_joints)
    burt.translatel_rel([0,0,-0.03])
    burt.wait_for_gripper()
    burt.close_gripper(5)
    burt.close_gripper(5)
    burt.force_move([0,0,-0.3])
    burt.translatel_rel([0,0,0.1])

    # move to passing waypoints
    burt.movej(wp.burt_passj,wait=False)
    urnie.movej(wp.urnie_passj,wait=False)

    # align robots
    burt_pose = burt.getl()
    xy = urnie.calc_pose(burt_pose)
    demand_pose = [xy[0]-0.03,xy[1],burt_pose[2]]
    urnie.translatel(demand_pose)
    urnie.wait_for_gripper()
    urnie.force_move([0.05,0,0],force=40)
    #urnie.translatel_rel([-0.001,0,0])

    # pass part
    urnie.open_gripper(2)
    #urnie.open_gripper(2)
    burt.open_gripper(1)
    burt.open_gripper(1)
    burt.open_gripper(10,wait=False)

    # move robots away
    urnie.translatel_rel([-0.05,0,0])
    burt.home(wait=False)
    urnie.home()

    # insert part into taskboard
    urnie.movejl(self.board.calc_pose([0.06, 0.34, 0.04, 0, -pi, 0]))
    urnie.insert()

    # release part
    urnie.close_gripper(2)
    #urnie.close_gripper(2)
    urnie.translatel_rel([0,0,0.05])
    urnie.open_gripper(2)
    urnie.cal_bearing(wait=False)

    # home
    urnie.home()
    return False

def part_2(burt,urnie,coords): # coords: [x,y]
    """
    # Target Parts: 6mm bearing retainer pin
    # Task: Insersion into a hole
    # Points: 4
    # Notes: 2 hands to turn over part
    """
    # open and orient fingers
    urnie.open_gripper(2,wait=False)
    urnie.close_gripper(2,wait=False)
    burt.open_gripper(2,wait=False)

    # home robots
    urnie.home(wait=False)
    burt.home(wait=False)

    # pick part with burt
    demand_joints = burt.get_inverse_kin([coords[0],coords[1],0.045,0,pi,0])
    demand_joints[5] += pi/2
    burt.movej(demand_joints)
    burt.translatel_rel([0,0,-0.025])
    burt.wait_for_gripper()
    burt.close_gripper(7)
    burt.close_gripper(7)
    burt.translatel_rel([0,0,0.06])

    # move to passing waypoints
    burt.movej(wp.burt_passj,wait=False)
    urnie.movej(wp.urnie_passj,wait=False)

    # align robots
    burt_pose = burt.getl()
    xy = urnie.calc_pose(burt_pose)
    demand_pose = [xy[0]-0.04,xy[1],burt_pose[2]]
    urnie.translatel(demand_pose)
    urnie.wait_for_gripper()
    urnie.translatel_rel([0.04,0,0])

    # pass part
    urnie.close_gripper(3)
    urnie.close_gripper(3)
    burt.open_gripper(1)
    burt.open_gripper(5,wait=False)

    # move robots away
    urnie.translatel_rel([-0.05,0,0])
    burt.home(wait=False)
    urnie.home()

    # insert part into taskboard
    urnie.movejl(board.calc_pose([0.05, 0.05, 0.035, 0, -pi, 0]))
    urnie.insert()

    # release par and home
    urnie.open_gripper(1)
    urnie.open_gripper(5,wait=False)
    urnie.home()
    return False

def part_3(burt,urnie,coords): # coords: [x,y]
    """
    # Target Parts: 17mm spacer for bearings
    # Task: Insersion into a hole
    # Points: 4
    """
    # open and orient fingers
    urnie.open_gripper(4,wait=False)

    # home
    urnie.home()

    # move to spacer
    urnie.move_ors([coords[0],coords[1],0.06,0,pi,0])
    urnie.wait_for_gripper()
    urnie.movel_tool([0,0,0.028,0,0,0])

    # pick up spacer
    urnie.close_gripper(3)
    urnie.force_move([0, 0, -0.1],force=50)
    urnie.movel_tool([0,0,-0.02,0,0,0])

    # home
    urnie.home()

    # move to hole
    urnie.movejl(board.calc_pose([0.09, 0.08, 0.03, 0, -pi, 0]))
                
    # insert
    urnie.insert()

    # release
    urnie.open_gripper(1)
    urnie.open_gripper(5,wait=False)

    # return home
    urnie.home()
    return False

def part_4(burt,urnie,coords): # coords: [x,y]
    """
    # Target Parts: 9mm spacer for bearings
    # Task: Insersion into a hole
    # Points: 4
    """
    # open and orient fingers
    urnie.open_gripper(3,wait=False)

    # home
    urnie.home()

    # move to spacer
    urnie.move_ors([coords[0],coords[1],0.06,0,pi,0])
    urnie.wait_for_gripper()
    urnie.movel_tool([0,0,0.028,0,0,0])

    # pick up spacer
    urnie.close_gripper(4)
    urnie.force_move([0, 0, -0.1],force=50)
    urnie.movel_tool([0,0,-0.02,0,0,0])

    # home
    urnie.home()

    # move to hole
    urnie.movejl(board.calc_pose([0.12, 0.15, 0.03, 0, -pi, 0]))
                
    # insert
    urnie.insert()

    # release
    urnie.open_gripper(1)
    urnie.open_gripper(5,wait=False)

    # return home
    urnie.home()
    return False

def part_5(burt,urnie,coords): # coords: [x,y]
    """
    # Target Parts: 10mm rotary shaft
    # Task: Conjunction with a tapped hole
    # Points: 8
    """
    # open and orient fingers
    urnie.open_gripper(3,wait=False)

    # home
    urnie.home()

    # move to rotary shaft
    urnie.move_ors([coords[0],coords[1],0.085,0,pi,0])
    urnie.wait_for_gripper()
    urnie.movel_tool([0,0,0.028,0,0,0])

    # pick up rotary shaft
    urnie.close_gripper(3)
    urnie.force_move([0, 0, -0.1],force=50)
    urnie.movel_tool([0,0,-0.02,0,0,0])

    # home
    urnie.home()

    # move to thread
    urnie.movejl(board.calc_pose([0.14, 0.07, 0.065, 0, -pi, 0]))
                
    # insert
    urnie.screw_in(4,R=15)

    # release
    urnie.open_gripper(1)
    urnie.open_gripper(5,wait=False)

    # return home
    urnie.home()
    return False

def part_6(burt,urnie,coords): # coords: [x,y]
    """
    # Target Parts: 4mm round belt
    # Task: Looping over pulleys
    # Points: 10
    """
    # open gripper
    burt.open_gripper(3,wait=False)

    # home burt
    burt.home(wait=False)

    # move over belt
    burt.movejl([coords[0]-0.07,coords[1],0.025,0,pi,0])
    burt.wait_for_gripper()

    # pick up belt
    burt.force_move([0,0,-0.1])
    burt.translatel_rel([0,0,0.0015])
    burt.close_gripper(0)
    burt.close_gripper(7)
    burt.close_gripper(7)
    burt.translatel_rel([0,0,0.2])

    # move to first pulley
    taskboard_pose = urnie.board.calc_pose([0.34, 0.188, 0.16, 0, -pi, 0])
    xy = burt.calc_inverse(taskboard_pose)
    demand_pose = [xy[0],xy[1],taskboard_pose[2],taskboard_pose[3],taskboard_pose[4],taskboard_pose[5]]
    demand_joints = burt.get_inverse_kin(demand_pose)
    demand_joints[5]-=pi/2
    burt.movej(demand_joints)

    # hook and move to second pulley
    burt.translatel_rel([0,0,-0.12])
    taskboard_pose = urnie.board.calc_pose([0.34, 0.06, 0.04, 0, -pi, 0])
    xy = burt.calc_inverse(taskboard_pose)
    burt.translatel([xy[0],xy[1],taskboard_pose[2]])

    # hook over second pulley
    burt.movel_tool([0,0,0,0,pi/18.0+pi/36.0,0])
    burt.force_move([0,-0.025,0],force=50)
    burt.translatel_rel([0,0,-0.005])
    burt.force_move([0,0,-0.1],force=40)
    burt.translatel_rel([0,-0.003,0.001])
    burt.movel_tool([0,0,0,0,-pi/18.0-pi/36.0,0])
    burt.movel_tool([0.003,0,0.003,0,0,0])
    burt.movel_tool([0,0,0,0,-pi/9.0,0])
    burt.open_gripper(1)
    burt.open_gripper(1)
    #burt.movel_tool([0,0,0,0,pi/9.0,0])
    #burt.translatel_rel([0,0,0.05])
    #burt.movel_tool([0,0,0,0,pi/9.0,0])
    demand_joints = burt.getj()
    demand_joints[1]+=pi/18
    burt.movej(demand_joints)
    burt.open_gripper(7,wait=False)

    # home
    burt.home()
    return False

def part_7(burt,urnie,coords_bolt,coords_nut): # coords: [x,y]
    """
    # Target Parts: M6 nut and M6 bolt
    # Task: Fasten a nut and bolt
    # Points: 10
    # Notes: 2 hands to turn over bolt, then 2nd hand picks up nut
    """
    # open and orient fingers
    urnie.open_gripper(3,wait=False)
    burt.open_gripper(3,wait=False)

    # home robots
    urnie.home(wait=False)
    burt.home(wait=False)

    # pick M6 bolt with burt
    demand_joints = burt.get_inverse_kin([coords_bolt[0],coords_bolt[1],0.025,0,pi,0])
    demand_joints[5] += pi/2
    burt.movej(demand_joints)
    burt.wait_for_gripper()
    burt.force_move([0, 0, -0.1],force=50)
    burt.translatel_rel([0,0,0.012])
    burt.close_gripper(7)
    burt.close_gripper(7)
    burt.translatel_rel([0,0,0.06])

    # move to passing waypoints
    burt.movej(wp.burt_passj,wait=False)
    urnie.movej(wp.urnie_passj,wait=False)

    # align robots
    burt_pose = burt.getl()
    xy = urnie.calc_pose(burt_pose)
    urnie.translatel([xy[0]-0.03,xy[1],burt_pose[2]])
    urnie.wait_for_gripper()
    urnie.translatel_rel([0.03,0,0])

    # pass part
    urnie.close_gripper(4)
    #urnie.close_gripper(2)
    burt.open_gripper(1)
    burt.open_gripper(1)
    burt.open_gripper(5,wait=False)

    # move robots away
    urnie.translatel_rel([-0.045,0,0])
    urnie.home(wait=False)
    burt.home()

    # move urnie to bracket
    urnie.movej(wp.urnie_bracketj)
    urnie.translatejl(urnie.board.calc_pose([0.344,0.385,0.05,0,pi,0]),wait=False)
    urnie.translatejl_rel([0,-0.01,0])

    # pick up M6 nut with burt
    demand_joints = burt.get_inverse_kin([coords_nut[0],coords_nut[1],0.02,0,pi,0])
    demand_joints[5] += pi/2
    burt.movej(demand_joints)
    burt.wait_for_gripper()
    burt.force_move([0, 0, -0.1],force=50)
    burt.translatel_rel([0,0,0.003])
    burt.close_gripper(0)
    burt.close_gripper(7)
    burt.close_gripper(7)
    burt.translatel_rel([0,0,0.06])

    # move to bracket
    burt.home()
    burt.movej(wp.burt_bracketj)
    taskboard_pose = urnie.board.calc_pose([0.343,0.358,0.053,0,pi,0])
    xy = burt.calc_inverse(taskboard_pose)
    burt.translatejl([xy[0],xy[1],taskboard_pose[2]])

    # insert part into taskboard
    urnie.insert([0,-0.08,0],max_force=40,R=4)
    urnie.screw_in(6,R=15)

    # release part
    urnie.open_gripper(1,wait=False)
    burt.open_gripper(1)
    urnie.open_gripper(5,wait=False)
    burt.open_gripper(5,wait=False)
    
    # move away from bracket
    urnie.translatejl_rel([0,0.02,0],wait=False)
    burt.translatejl_rel([0,-0.02,0])
    burt.movej(wp.burt_bracketj,wait=False)
    urnie.movej(wp.urnie_bracketj)

    # home
    burt.home()
    urnie.home()
    return False

def part_8(burt,urnie,coords): # coords: [x,y,theta]
    """
    # Target Parts: M12 nut
    # Task: Fasten a nut and bolt
    # Points: 8
    """
    # open and orient fingers
    urnie.cal_fingers(wait=False)
    urnie.open_gripper(5,wait=False)
    urnie.open_gripper(5,wait=False)

    # home
    urnie.home()

    # move to nut
    demand_joints = urnie.get_inverse_kin(urnie.calc_pose([coords[0],coords[1],0.021,0,pi,0]))
    demand_joints[5] += coords[2]
    urnie.movej(demand_joints)
    urnie.wait_for_gripper()
    urnie.movel_tool([0,0,0.02,0,0,0])

    # pick up nut
    urnie.close_gripper(3)
    urnie.close_gripper(3)

    # home
    urnie.home()

    # move to bolt
    urnie.movejl(urnie.board.calc_pose([0.15, 0.320, 0.0125, 0, -pi, 0]))
                
    # screw in
    urnie.screw_in(12)

    # release
    urnie.open_gripper(1)
    urnie.open_gripper(5,wait=False)

    # return home
    urnie.home()
    return False

def part_9(burt,urnie,coords): # coords: [x,y]
    """
    # Target Parts: M6 washer
    # Task: Placing onto a bolt 
    # Points: 5
    """
    # open and orient fingers
    urnie.cal_bearing(wait=False)
    urnie.close_gripper(1,wait=False)

    # home
    urnie.home()

    # move to washer
    urnie.move_ors([coords[0],coords[1],0.01,0,pi,0])
    urnie.wait_for_gripper()
    urnie.force_move([0, 0, -0.1],force=50)
    urnie.translatel_rel([0,0,0.0017])

    # pick up washer
    urnie.close_gripper(3)
    urnie.close_gripper(3)

    # home
    urnie.home()

    # move to bolt
    urnie.movejl(urnie.board.calc_pose([0.25, 0.26, 0.025, 0, -pi, 0]))
                
    # lower
    urnie.insert(axis=[0,0,-0.025])

    # release
    urnie.open_gripper(1)
    urnie.open_gripper(5,wait=False)

    # return home
    urnie.home()
    return False

def part_10(burt,urnie,coords): # coords: [x,y]
    """
    # Target Parts: M10 washer
    # Task: Placing onto a shaft 
    # Points: 5
    """
    # open and orient fingers
    urnie.open_gripper(3,wait=False)

    # home
    urnie.home()

    # move to washer
    urnie.move_ors([coords[0],coords[1],0.01,0,pi,0])
    urnie.wait_for_gripper()
    urnie.force_move([0, 0, -0.1],force=50)
    urnie.translatel_rel([0,0,0.002])

    # pick up washer
    urnie.close_gripper(2)
    urnie.close_gripper(2)

    # home
    urnie.home()

    # move to bolt
    urnie.movejl(urnie.board.calc_pose([0.04, 0.16, 0.055, 0, -pi, 0]))
                
    # lower
    urnie.insert()

    # release
    urnie.open_gripper(1)
    urnie.open_gripper(5,wait=False)

    # return home
    urnie.home()
    return False

def part_11(burt,urnie):
    """
    # Target Parts: M3 setscrew
    # Task: Screwing into a tapped hole
    # Points: 10
    """
    # get allen key with urnie
    get_allen_key(urnie,'m2')

    # move to setscrew
    urnie.translatejl(urnie.board.calc_pose([0.221, 0.059, 0.045, 0, -pi, 0]))

    # insert allen key into set screw
    urnie.insert(hunting_radius=0.0005,R=4)

    # screw in
    urnie.screw_in(3,R=10)

    # move up
    urnie.translatel_rel([0,0,0.05])

    # stow allen key
    stow_allen_key(urnie,'m2')

    # home
    urnie.home()
    return False

def part_12(burt,urnie,coords): # coords: [x,y]
    """
    # Target Parts: M3 bolt
    # Task: Screwing into a tapped hole
    # Points: 10
    # Notes: 2 hands to turn over part
    """
    # get allen key with urnie 
    global urnie_flag
    _thread.start_new_thread(get_allen_key,(urnie,'m3'))
    # open and orient fingers
    burt.open_gripper(2,wait=False)

    # home burt
    burt.home(wait=False)

    # pick M3 bolt with burt
    demand_joints = burt.get_inverse_kin([coords[0],coords[1],0.025,0,pi,0])
    demand_joints[5] += pi/2
    burt.movej(demand_joints)
    burt.wait_for_gripper()
    burt.force_move([0, 0, -0.1],force=50)
    burt.translatel_rel([0,0,0.005])
    burt.close_gripper(7)
    burt.close_gripper(7)
    burt.force_move([0, 0, -0.01],force=50)
    burt.close_gripper(1)
    burt.translatel_rel([0,0,0.06])

    # move to passing waypoints
    burt.movej(wp.burt_passj,wait=False)
    while urnie_flag == True:
        time.sleep(0.1)
    urnie.movej(wp.urnie_passj,wait=False)

    # align robots
    burt_pose = burt.getl()
    xy = urnie.calc_pose(burt_pose)
    urnie.translatel([xy[0]-0.06,xy[1],burt_pose[2]])

    # pass part
    urnie.insert([0.03,0,0],hunting_radius=0.0005,R=6)
    urnie.force_move([0.01,0,0])
    burt.open_gripper(1)
    burt.open_gripper(1)
    burt.open_gripper(10,wait=False)

    # move robots away
    urnie.translatel_rel([-0.045,0,0])
    burt.home(wait=False)

    # insert part into taskboard
    tcp = copy.deepcopy(wp.rotary_tcp)
    tcp[2]+=0.03
    urnie.set_tcp(tcp)
    urnie.translatejl(urnie.board.calc_pose([0.262, 0.139, 0.12, 0, -pi, 0]))
    urnie.movel_tool([0,0,0,0,pi/2,0])
    urnie.set_tcp(wp.rotary_tcp)
    urnie.translatejl_rel([0,0,-0.08])
    #urnie.force_move([0, 0, -0.1],force=50)
    #urnie.translatel_rel([0,0,0.001])
    urnie.insert([0,0,-0.03],hunting_radius=0.001,R=6)
    urnie.screw_in(3,R=15)

    # stow allen key
    urnie.translatel_rel([0,0,0.05])
    stow_allen_key(urnie,'m3')

    # home robots
    urnie.home(wait=False)
    burt.home()
    return False

def part_13(burt,urnie,coords): # coords: [x,y]
    """
    # Target Parts: M4 bolt
    # Task: Screwing into a tapped hole
    # Points: 10
    # Notes: 2 hands to turn over part
    """
    # Get allen key with urnie
    global urnie_flag
    _thread.start_new_thread(get_allen_key,(urnie,'m4'))
    # open and orient fingers
    burt.open_gripper(2,wait=False)

    # home burt
    burt.home(wait=False)

    # pick M4 bolt with burt
    demand_joints = burt.get_inverse_kin([coords[0],coords[1],0.025,0,pi,0])
    demand_joints[5] += pi/2
    burt.movej(demand_joints)
    burt.wait_for_gripper()
    burt.force_move([0, 0, -0.1],force=50)
    burt.translatel_rel([0,0,0.006])
    burt.close_gripper(7)
    burt.close_gripper(7)
    burt.force_move([0, 0, -0.01],force=50)
    burt.close_gripper(1)
    burt.translatel_rel([0,0,0.06])

    # move to passing waypoints
    burt.movej(wp.burt_passj,wait=False)
    while urnie_flag == True:
        time.sleep(0.1)
    urnie.movej(wp.urnie_passj,wait=False)

    # align robots
    burt_pose = burt.getl()
    xy = urnie.calc_pose(burt_pose)
    urnie.translatel([xy[0]-0.07,xy[1],burt_pose[2]])

    # pass part
    urnie.insert([0.03,0,0],hunting_radius=0.0005,R=4)
    urnie.force_move([0.01,0,0])
    burt.open_gripper(1)
    burt.open_gripper(3)
    burt.open_gripper(10,wait=False)

    # move robots away
    urnie.translatel_rel([-0.045,0,0])
    burt.home(wait=False)

    ipt = input("continue?(y/n)")
    if ipt == 'n':
        stow_allen_key(urnie,'m4')
        return False

    # insert part into taskboard
    tcp = copy.deepcopy(wp.rotary_tcp)
    tcp[2]+=0.03
    urnie.set_tcp(tcp)
    urnie.translatejl(urnie.board.calc_pose([0.28, 0.35, 0.1, 0, -pi, 0]))
    urnie.movel_tool([0,0,0,0,pi/2,0])
    urnie.set_tcp(wp.rotary_tcp)
    urnie.translatejl_rel([0,0,-0.06])
    urnie.force_move([0, 0, -0.1],force=50)
    urnie.translatel_rel([0,0,0.001])
    urnie.screw_in(4,R=20)

    # stow allen key
    urnie.translatel_rel([0,0,0.05])
    stow_allen_key(urnie,'m4')

    # home robots
    urnie.home(wait=False)
    burt.home()
    return False

def part_14(burt,urnie,coords): # coords: [x,y]
    """
    # Target Parts: Pulley
    # Task: Placing onto a shaft
    # Points: 4
    """
    # open and orient fingers
    burt.cal_gripper(wait=False)

    # home
    burt.home()

    # move to pulley
    demand_joints = burt.get_inverse_kin([coords[0],coords[1],0.02,0,pi,0])
    demand_joints[5] += pi/2
    burt.movej(demand_joints)
    burt.wait_for_gripper()
    burt.force_move([0, 0, -0.1],force=50)
    burt.translatel_rel([0,0,0.004])

    # pick up pulley
    burt.close_gripper(0)
    burt.close_gripper(7)
    burt.close_gripper(7)
    burt.translatel_rel([0,0,0.1])

    # move to shaft
    taskboard_pose = urnie.board.calc_pose([0.17, 0.23, 0.075, 0, -pi, 0])
    xy = burt.calc_inverse(taskboard_pose)
    demand_pose = [xy[0],xy[1],taskboard_pose[2],taskboard_pose[3],taskboard_pose[4],taskboard_pose[5]]
    burt.movejl(demand_pose)
                
    # lower
    burt.insert(max_force=35)

    # release
    burt.open_gripper(1)
    burt.open_gripper(10,wait=False)
    burt.translatel_rel([0,0,0.02])

    # return home
    burt.home()
    return False

def part_15(burt,urnie,coords): # coords: [x,y]
    """
    # Target Parts: M10 end cap
    # Task: Placing onto a shaft
    # Points: 4
    # Notes: 2 hands to turn over part?
    """
    # open and orient fingers
    urnie.cal_gripper(wait=False)
    burt.open_gripper(2,wait=False)
    urnie.close_gripper(2,wait=False)

    # home
    urnie.home(wait=False)
    burt.home(wait=False)

    # pick end cap with burt
    demand_joints = burt.get_inverse_kin([coords[0],coords[1],0.025,0,pi,0])
    demand_joints[5] += pi/2
    burt.movej(demand_joints)
    burt.wait_for_gripper()
    burt.force_move([0, 0, -0.1],force=50)
    burt.translatel_rel([0,0,0.006])
    burt.close_gripper(7)
    burt.close_gripper(7)
    burt.translatel_rel([0,0,0.06])

    # move to passing waypoints
    burt.movej(wp.burt_passj,wait=False)
    urnie.movej(wp.urnie_passj,wait=False)

    # align robots
    burt_pose = burt.getl()
    xy = urnie.calc_pose(burt_pose)
    urnie.translatel([xy[0]-0.02,xy[1],burt_pose[2]])

    # pass part
    urnie.force_move([0.03,0,0])
    urnie.close_gripper(3)
    urnie.close_gripper(0)
    burt.open_gripper(1)
    burt.open_gripper(2)
    burt.open_gripper(10,wait=False)

    # move robots away
    urnie.translatel_rel([-0.045,0,0.05])
    burt.home(wait=False)

    # insert part into taskboard
    urnie.movejl(urnie.board.calc_pose([0.04, 0.16, 0.07, 0, -pi, 0]))
    urnie.insert(max_force=40)
    urnie.open_gripper(1)
    urnie.open_gripper(5,wait=False)

    # stow allen key
    urnie.translatel_rel([0,0,0.05])

    # home robots
    urnie.home(wait=False)
    burt.home()
    return False