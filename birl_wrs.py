import waypoints as wp
import time
import copy
import serial
from math import pi
import numpy
import json
import socket


import PoseEstimation as pe
import kg_robot as kgr
import dual_robot_cal as drc
import taskboard as tb
import assembly as ab


def main():
    print("------------Configuring Burt-------------\r\n")
    #burt = kgr.kg_robot(ee_port="COM34",side='left')
    #burt = kgr.kg_robot(port=30010,side='left')
    #burt = kgr.kg_robot(port=30010,ee_port="COM32",side='left',db_host="129.169.80.10")
    #burt = kgr.kg_robot()
    print("----------------Hi Burt!-----------------\r\n\r\n")
    
    #print("------------Configuring Urnie------------\r\n")
    urnie = kgr.kg_robot(port=30010,db_host="192.168.1.10",ee_port="COM32")
    #urnie = kgr.kg_robot(port=30000, side='right')
    #urnie = kgr.kg_robot(port=30000,ee_port="COM32", side='right',db_host="192.168.1.20")
    #urnie = kgr.kg_robot()
    #print("----------------Hi Urnie!----------------\r\n\r\n")

    try:
        with open('placement_locations.json') as f:
            placement_locations = json.load(f)

        while 1:
            ipt = input("cmd: ")
            if ipt == 'close':
                break
            elif ipt == 'home':
                urnie.home()
                #burt.home()
            elif ipt == 'cal':
                #drc.cal_dual_robots(burt,urnie)
                #tb.calibrate(urnie)
                #tb.cal_allen_keys(urnie)
                ab.calibrate(urnie)
                ab.cal_allen_keys(urnie)
                ab.cal_trays(urnie)
            elif ipt == 't':
                urnie.movel_tool([0,0,0,pi/18.0,0,0])
                #burt.to_switch()
                #burt.movej(wp.burt_pulleyj)
                #burt.translatel_rel([0,0,-0.07])
                #burt.wait_for_gripper()
                #burt.force_move([0,0.05,0],force=60)
                #burt.move_ors(urnie.pedestal.calc_pose([0.12,0.2,0.005,0,pi,0]),wait=False)
                #urnie.cal_fingers(wait=False)
                #urnie.movej(wp.urnie_pedestalj)
                #urnie.translatel_pedestal([-0.025,0.036,0.019])
                #urnie.translatel_pedestal([-0.025,0.036,0.063])
                #urnie.translatel_pedestal([-0.025,0.058,0.041])
                #urnie.force_move([0.05,0,0],force=30)
                #urnie.translatel_rel([-0.003,0,0])

            # taskboard tests
            elif ipt == '1':
                print("burt x, y location of Bearings with Housing: {}".format(placement_locations["housing"]))
                input("Continue?")
                tb.part_1(burt,urnie,[placement_locations["housing"][0],placement_locations["housing"][1]])

            elif ipt == '2':
                print("burt x, y location of Bearings with Bearing Pin: {}".format(placement_locations["Bearing_pin"]))
                input("Continue?")
                tb.part_2(burt,urnie,[placement_locations["Bearing_pin"][0],placement_locations["Bearing_pin"][1]])

            elif ipt == '3':
                print("burt x, y location of 17mm_Spacer: {}".format(placement_locations["17mm_Spacer"]))
                input("Continue?")
                tb.part_3(burt,urnie,[placement_locations["17mm_Spacer"][0],placement_locations["17mm_Spacer"][1]])

            elif ipt == '4':
                print("burt x, y location of 9mm Spacer: {}".format(placement_locations["9mm_Spacer"]))
                input("Continue?")
                tb.part_4(burt,urnie,[placement_locations["9mm_Spacer"][0],placement_locations["9mm_Spacer"][1]])

            elif ipt == '5':
                print("burt x, y location of Rotary Shaft: {}".format(placement_locations["Rotary_Shaft"]))
                input("Continue?")
                tb.part_5(burt,urnie,[placement_locations["Rotary_Shaft"][0],placement_locations["Rotary_Shaft"][1]])

            elif ipt == '6':
                print("burt x, y location of Belt: {}".format(placement_locations["belt"]))
                input("Continue?")
                tb.part_6(burt,urnie,[placement_locations["belt"][0],placement_locations["belt"][1]])

            elif ipt == '7':
                print("burt x, y location of M6 Bolt: {}".format(placement_locations["M6_screw"]))
                print("burt x, y location of M6 Nut: {}".format(placement_locations["M6_nut"]))
                input("Continue?")
                tb.part_7(burt,urnie,[placement_locations["M6_screw"][0],placement_locations["M6_screw"][1]],[placement_locations["M6_nut"][0],placement_locations["M6_nut"][1]])

            elif ipt == '8':
                print("burt x, y location of M12 Nut: {}".format(placement_locations["M12_Nut"]))
                input("Continue?")
                tb.part_8(burt,urnie,[placement_locations["M12_Nut"][0],placement_locations["M12_Nut"][1],pi/6])

            elif ipt == '9':
                print("burt x, y location of Small Washer: {}".format(placement_locations["Small_washer"]))
                input("Continue?")
                tb.part_9(burt,urnie,[placement_locations["Small_washer"][0],placement_locations["Small_washer"][1]])

            elif ipt == '10':
                print("burt x, y location of Large Washer: {}".format(placement_locations["Large_Washer"]))
                input("Continue?")
                tb.part_10(burt,urnie,[placement_locations["Large_Washer"][0],placement_locations["Large_Washer"][1]])

            elif ipt == '11':
                input("Continue?")
                tb.part_11(burt,urnie,board)

            elif ipt == '12':
                print("burt x, y location of M3 Bolt: {}".format(placement_locations["M3_Bolt"]))
                input("Continue?")
                tb.part_12(burt,urnie,[placement_locations["M3_Bolt"][0],placement_locations["M3_Bolt"][1]])

            elif ipt == '13':
                print("burt x, y location of M4 Bolt: {}".format(placement_locations["M4_Bolt"]))
                input("Continue?")
                tb.part_13(burt,urnie,[placement_locations["M4_Bolt"][0],placement_locations["M4_Bolt"][1]])

            elif ipt == '14':
                print("burt x, y location of Pulley: {}".format(placement_locations["pulley"]))
                input("Continue?")
                tb.part_14(burt,urnie,[placement_locations["pulley"][0],placement_locations["pulley"][1]])

            elif ipt == '15':
                print("burt x, y location of End Cap: {}".format(placement_locations["End_Cap"]))
                input("Continue?")
                tb.part_15(burt,urnie,[placement_locations["End_Cap"][0],placement_locations["End_Cap"][1]])

            # assembly tests
            elif ipt == 'a':
                print("Assemble motor")
                input("Continue?")
                ab.subtask_a(burt,urnie,0)

            elif ipt == 'b':
                print("Assemble motor pulley")
                input("Continue?")
                ab.subtask_b(burt,urnie,0)

            elif ipt == 'c':
                print("Assemble output shaft")
                input("Continue?")
                ab.subtask_c(burt,urnie,0)

            elif ipt == 'd':
                print("Assemble output pulley")
                input("Continue?")
                ab.subtask_d(burt,urnie,0)

            elif ipt == 'e':
                print("Assemble tensioner shaft")
                input("Continue?")
                ab.subtask_e(burt,urnie,0)

            elif ipt == 'f':
                print("Assemble motor fixing plate")
                input("Continue?")
                ab.subtask_f(burt,urnie,0)

            elif ipt == 'g':
                print("Assemble output fixing plate")
                input("Continue?")
                ab.subtask_g(burt,urnie,0)

            elif ipt == 'h':
                print("Assemble belt")
                input("Continue?")
                ab.subtask_h(burt,urnie,0)
            else:
                var = int(input("var: "))
                urnie.serial_send(ipt,var,True)

        


    finally:
        print("Goodbye")
        #burt.close()
        urnie.close()
if __name__ == '__main__': main()