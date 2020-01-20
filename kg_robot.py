import numpy as np
import time
import serial
import scipy.optimize
import socket
import math
from math import pi

import waypoints as wp
import taskboard as tb
import assembly as ab

class kg_robot():
    def __init__(self, port=False, ee_port=False, side='left',db_host=False):
        self.port = port
        self.ee_port = ee_port
        self.side = side
        self.db_host = db_host
        #self.calc_offset()
        if db_host!=False:
            self.dashboard = kg_robot_dashboard(host=self.db_host)
            self.dashboard.init()

        #init ur5 connection
        self.open=False
        if port!=False:
            self.host = "192.168.1.5"

            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((self.host, self.port)) # Bind to the port
            s.listen(5) # Now wait for client connection.
            self.c, self.addr = s.accept() # Establish connection with client.
            print("Connected to",self.side,"robot\r\n")
            self.open=True

            if self.side == 'right':
                self.board = tb.taskboard()
                self.pedestal = ab.assembly()
                self.home(pose = wp.urnie_homej, wait=False)
            elif self.side == 'left':
                self.home(pose = wp.burt_homej, wait=False)


        #init gripper connection and update robot tcp
        if ee_port!=False:
            self.ee = serial.Serial(self.ee_port, 9600)  # open serial port
            while self.ee.isOpen()==False:
                print("Waiting for hand")
            #print("Serial port opened :)")

            self.ee.send_break()
            time.sleep(1) # This is needed to allow MBED to send back command in time!
            ipt = bytes.decode(self.ee.readline())
            print("Connected to",ipt)

            if port!=False:
                if ipt=="Rotary Gripper\r\n":
                    self.set_tcp(wp.rotary_tcp)
                    self.set_payload(1.8)
                elif ipt=="Pincher Gripper\r\n":
                    self.set_tcp(wp.pincher_tcp)
                    self.set_payload(0.5)
                elif ipt == "ElectroMag Gripper\r\n":
                    self.set_tcp(wp.magnet_tcp) 
                    self.set_payload(0.5)
                else:
                    print("NO GRIPPER DETECTED")

        return



    #-------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #
    #                                                                      Communications
    #
    #-------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def socket_send(self, prog):
        #if self.db_host != False:
        #    self.check_connection()

        msg = "No message from robot"
        try:
            # Send formatted CMD
            self.c.send(str.encode(prog))
            # Wait for reply
            if prog[-3]=='0':
                msg=bytes.decode(self.c.recv(1024))
                if msg=="No message from robot" or msg=='':
                    print(".......................Robot disconnected :O.......................")
                    input("press enter to continue")

        except socket.error as socketerror:
            print(".......................Some kind of error :(.......................")
            input("press enter to continue")
            #if self.db_host != False:
            #    self.dashboard.reconnect(self.db_host)
            #    self.dashboard.init()
            #    self.socket_send(prog, wait=wait)
        return msg

    def format_prog(self,CMD,pose=[0,0,0,0,0,0],acc=0.1,vel=0.1,t=0,r=0,w=True):
        wait=0
        if w==False:
            wait=1
        return "({},{},{},{},{},{},{},{},{},{},{},{})\n".format(CMD,*pose,acc,vel,t,r,wait)

    def serial_send(self,cmd,var,wait):
        ipt = ""
        self.ee.reset_input_buffer()
        #self.ee.flush()
        self.ee.write(str.encode(cmd+chr(var+48)+"\n"))
        #wait for cmd acknowledgement
        while True:
            ipt = bytes.decode(self.ee.readline())
            #print("gripper data: ", ipt)
            if ipt == "received\r\n":
                break
        #wait for cmd completion
        if wait==True:
            while True:
                ipt = bytes.decode(self.ee.readline())
                #print("gripper data: ", ipt)
                if ipt == "done\r\n":
                    #print("Completed gripper CMD")
                    break
        return ipt

    def decode_msg(self,prog):
        msg = self.socket_send(prog)
        #print "recieved: ",msg

        # Decode Pose or Joints from UR
        current_position = [0,0,0,0,0,0]
        data_start = 0
        data_end = 0
        n = 0
        x = 0
        while x < len(msg):
            if msg[x]=="," or msg[x]=="]" or msg[x]=="e":
                data_end = x
                current_position[n] = float(msg[data_start:data_end])
                if msg[x]=="e":
                    current_position[n] = current_position[n]*math.pow(10,float(msg[x+1:x+4]))
                    #print "e", msg[x+1:x+4]
                    #print "e", int(msg[x+1:x+4])
                    if n < 5:
                        x = x+5
                        data_start = x
                    else:
                        break
                n=n+1
            if msg[x]=="[" or msg[x]==",":
                data_start = x+1
            x = x+1

        return current_position

    
    def close(self):
        """
        close connection to robot and stop internal thread
        """
        try:
            self.ee.reset_output_buffer()  # Close gripper
        except:
            # No gripper connected
            pass
        if self.open==True:
            prog = self.format_prog(100)
            print(self.socket_send(prog))
            self.c.close()
        
        if self.db_host!=False:
            if self.dashboard.open==True:
                #print(self.dashboard.socket_send("quit\n"))
                self.dashboard.c.close()

    def check_connection(self):
        msg = "No message from robot"
        try:
            self.c.send(str.encode("(101,0,0,0,0,0,0,0,0,0,0,0)"))
            msg=bytes.decode(self.c.recv(1024))
            if msg!="ready":
                self.c.close()
                self.dashboard.reconnect(self.db_host)
                self.dashboard.init()
                time.sleep(1)
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                s.bind((self.host, self.port)) # Bind to the port
                s.listen(5) # Now wait for client connection.
                self.c, self.addr = s.accept() # Establish connection with client.
                print("Connected to",self.side,"robot\r\n")
                self.open=True
                time.sleep(1)
        except socket.error as socketerror:
            self.c.close()
            self.dashboard.reconnect(self.db_host)
            self.dashboard.init()
            time.sleep(1)
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((self.host, self.port)) # Bind to the port
            s.listen(5) # Now wait for client connection.
            self.c, self.addr = s.accept() # Establish connection with client.
            print("Connected to",self.side,"robot\r\n")
            self.open=True
            time.sleep(1)

    #-------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #
    #                                                                       UR5 Commands
    #
    #-------------------------------------------------------------------------------------------------------------------------------------------------------------------
    # CMD       Description                                         Reply
    # 0         joint move in linear space                          confirmation
    # 1         move in joint space                                 confirmation
    # 2         pose move in linear space                           confirmation
    # 3         pose move relative to current position              confirmation
    # 4         force move in single axis                           confirmation
    #
    # 10        get current pose                                    pose
    # 11        get current jonts                                   joints
    # 12        get inverse kin of sent pose                        joints
    # 13        get transform from current pose to sent pose        pose
    # 14        get force vector                                    pose
    # 15        get force magnitude                                 float
    #
    # 20        set tool centre point (tcp)                         confirmation
    # 21        set payload                                         confirmation
    #
    # 100       close socket on robot                               confirmation

    def movejl(self, pose, acc=0.5, vel=0.5, min_time=0, radius=0, wait=True):
        """
        joint move in linear space
        """
        prog = self.format_prog(0,pose=pose,acc=acc,vel=vel,t=min_time,r=radius,w=wait)
        return self.socket_send(prog)

    def movej(self, joints, acc=0.5, vel=0.5, min_time=0, radius=0, wait=True):
        """
        move to joint positions
        """
        prog = self.format_prog(1,pose=joints,acc=acc,vel=vel,t=min_time,r=radius,w=wait)
        return self.socket_send(prog)

    def movej_rel(self, joints, acc=0.5, vel=0.5, min_time=0, radius=0, wait=True):
        """
        move joint positions by 'joints'
        """
        demand_joints = self.getj()
        for i in range(0,6):
            demand_joints[i]+=joints[i]
        prog = self.format_prog(1,pose=demand_joints,acc=acc,vel=vel,t=min_time,r=radius,w=wait)
        return self.socket_send(prog)

    def movel(self, pose, acc=0.1, vel=0.1, min_time=0, radius=0, wait=True):
        """
        pose move in linear space
        """
        prog = self.format_prog(2,pose=pose,acc=acc,vel=vel,t=min_time,r=radius,w=wait)
        return self.socket_send(prog)

    def home(self, pose=None, type='j', acc=0.5, vel=0.5, wait=True):
        """
        move to home position, default joint space
        """
        if type == 'j':
            if pose!=None:
                self.homej = pose
            prog = self.format_prog(1,pose=self.homej,acc=acc,vel=vel,w=wait)
        elif type == 'l':
            if pose!=None:
                self.homel = pose
            prog = self.format_prog(0,pose=self.homel,acc=acc,vel=vel,w=wait)
        return self.socket_send(prog)

    def translatel_rel(self, pose, acc=0.5, vel=0.5, min_time=0, radius=0, wait=True):
        """
        translate relative to position in linear space
        """
        self.demand_pose = self.getl()
        self.demand_pose[0]+=pose[0]
        self.demand_pose[1]+=pose[1]
        self.demand_pose[2]+=pose[2]
        return self.movel(self.demand_pose,acc=acc,vel=vel,min_time=min_time,radius=radius,wait=wait)

    def translatejl_rel(self, pose, acc=0.5, vel=0.5, min_time=0, radius=0, wait=True):
        """
        translate relative to position in linear space using joint move
        """
        self.demand_pose = self.getl()
        self.demand_pose[0]+=pose[0]
        self.demand_pose[1]+=pose[1]
        self.demand_pose[2]+=pose[2]
        return self.movejl(self.demand_pose,acc=acc,vel=vel,min_time=min_time,radius=radius,wait=wait)

    def rotate_rel(self, pose, acc=0.5, vel=0.5, min_time=0, radius=0, wait=True):
        """
        joint rotate relative to current position
        """
        self.demand_pose = self.getj()
        self.demand_pose[0] += pose[0]
        self.demand_pose[1] += pose[1]
        self.demand_pose[2] += pose[2]
        self.demand_pose[3] += pose[3]
        self.demand_pose[4] += pose[4]
        self.demand_pose[5] += pose[5]
        return self.movej(self.demand_pose,acc=acc,vel=vel,min_time=min_time,radius=radius,wait=wait)


    def translatel(self, pose, acc=0.5, vel=0.5, min_time=0, radius=0, wait=True):
        """
        translate to position in linear space
        """
        self.demand_pose = self.getl()
        self.demand_pose[0]=pose[0]
        self.demand_pose[1]=pose[1]
        self.demand_pose[2]=pose[2]
        return self.movel(self.demand_pose,acc=acc,vel=vel,min_time=min_time,radius=radius,wait=wait)

    def translatejl(self, pose, acc=0.5, vel=0.5, min_time=0, radius=0, wait=True):
        """
        translate to position in linear space using joint move
        """
        self.demand_pose = self.getl()
        self.demand_pose[0]=pose[0]
        self.demand_pose[1]=pose[1]
        self.demand_pose[2]=pose[2]
        return self.movejl(self.demand_pose,acc=acc,vel=vel,min_time=min_time,radius=radius,wait=wait)

    def movel_tool(self, pose, acc=0.5, vel=0.5, min_time=0, radius=0, wait=True):
        """
        linear move in tool space
        """
        prog = self.format_prog(3,pose=pose,acc=acc,vel=vel,t=min_time,r=radius,w=wait)
        return self.socket_send(prog)

    def move_ors(self, pose, acc=0.5, vel=0.5, min_time=0, radius=0, wait=True):
        """
        joint move to a position in the linear space of the other robot
        """
        if self.side=='right':
            #xy = self.calc_pose(pose)
            demand_pose = self.burt_to_urnie(pose)
        elif self.side=='left':
            #xy = self.calc_inverse(pose)
            demand_pose = self.urnie_to_burt(pose)
        #demand_pose = [xy[0],xy[1],pose[2]]
        return self.movejl(demand_pose, acc=acc, vel=vel, min_time=min_time, radius=radius, wait=wait)

    def follow(self, pose, acc=0.1, vel=0.1, min_time=0, radius=0, wait=True):
        """
        linear move to a position in the linear space of the other robot
        """
        if self.side=='right':
            #xy = self.calc_pose(pose)
            demand_pose = self.burt_to_urnie(pose)
        elif self.side=='left':
            #xy = self.calc_inverse(pose)
            demand_pose = self.urnie_to_burt(pose)
        #demand_pose = [xy[0],xy[1],pose[2]]
        return self.translatel(demand_pose, acc=acc, vel=vel, min_time=min_time, radius=radius, wait=wait)

    def force_move(self, axis, acc=0.05, vel=0.05, min_time=0, force=50, wait=True):
        """
        move along axis with a maximum force, e.g. axis = [dist,0,0]
        """
        prog = self.format_prog(4,pose=axis+[0,0,0],acc=acc,vel=vel,t=min_time,r=force,w=wait)
        return self.socket_send(prog)

    def getl(self):
        """
        get TCP position
        """
        prog = self.format_prog(10)
        return self.decode_msg(prog)

    def getj(self):
        """
        get joints position
        """
        prog = self.format_prog(11)
        return self.decode_msg(prog)

    def get_inverse_kin(self,pose):
        """
        get inverse kin of pose
        """
        prog = self.format_prog(12,pose=pose)
        return self.decode_msg(prog)

    def get_forces(self):
        """
        get x,y,z forces and rx,ry,rz torques
        """
        prog = self.format_prog(14)
        return self.decode_msg(prog)

    def get_force(self):
        """
        get force magnitude
        """
        prog = self.format_prog(15)
        return float(self.socket_send(prog))

    def set_tcp(self, tcp):
        """
        set robot tool centre point
        """
        self.tcp = tcp
        prog = self.format_prog(20,pose=tcp)
        return self.socket_send(prog)

    def set_payload(self, weight, cog=None):
        """
        set payload in Kg
        cog is a vector x,y,z
        if cog is not specified, then tool center point is used
        """
        if cog==None:
            prog = self.format_prog(21,pose=self.tcp,acc=weight)
        else:
            prog = self.format_prog(21,pose=cog.extend([0,0,0]),acc=weight)
        return self.socket_send(prog)


    #-------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #
    #                                                                     Gripper Commands
    #
    #-------------------------------------------------------------------------------------------------------------------------------------------------------------------
    # CMD   Description
    # C     close gripper, larger var waits longer before timing out
    # O     open gripper, larger var waits longer before timing out
    # R     rotate gripper cw, var = no. rotations
    # U     rotate gripper ccw, var = no. rotations
    # G     calibrate finger rotation and bearing position
    # F     calibrate finger rotation
    # B     calibrate bearing position
    # H     switch electromagnet to hold
    # R     release electromagnet to drop objects    

    def wait_for_gripper(self):
        """
        wait for current gripper processes to finish
        """
        self.serial_send("W",0,True)
        return

    def close_gripper(self,var=0,wait=True):
        """
        close gripper, times out after ~var seconds
        """
        self.serial_send("C",var,wait)
        return

    def open_gripper(self,var=0,wait=True):
        """
        open gripper, times out after ~5*var seconds, if var>=5 calibrate open position instead
        """
        if var>=5 and self.side=='right':
            self.serial_send("B",0,wait)
        else:
            self.serial_send("O",var,wait)
        return

    def rotate_gripper_cw(self,var=0,wait=True):
        """
        rotate gripper cw var times, times out after ~var seconds
        """
        self.serial_send("R",var,wait)
        return

    def rotate_gripper_ccw(self,var=0,wait=True):
        """
        rotate gripper ccw var times, times out after ~var seconds
        """
        self.serial_send("U",var,wait)
        return

    def rotate_gripper_cont(self,var=0):
        """
        rotate gripper cw with 10*var% power, ccw with -ve power, continues to rotate until another cmd is sent
        """
        self.serial_send("S",52+var,True)
        return

    def cal_gripper(self,wait=True):
        """
        rotate fingers to swtch position and open gripper fully
        """
        self.serial_send("G",0,wait)
        return

    def cal_fingers(self,wait=True):
        """
        rotate fingers to swtch position
        """
        self.serial_send("F",0,wait)
        return

    def cal_bearing(self,wait=True):
        """
        open gripper fully
        """
        self.serial_send("B",0,wait)
        return

    def em_hold(self, wait=True):
        """
        turn on electromagnet
        """
        self.serial_send("H", 0, wait)
        return

    def em_release(self, wait=True):
        """
        turn off electromagnet
        """
        self.serial_send("R", 0, wait)
        return  

    def to_switch(self, wait=True):
        """
        move pincher gripper to switch boundary
        """
        self.serial_send("S", 0, wait)
        return

    #-------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #
    #                                                                      Combined Functions
    #
    #-------------------------------------------------------------------------------------------------------------------------------------------------------------------

    def insert(self,axis=[0,0,-0.05],max_force=50,hunting_radius=0.001,iter=50,circles=2,R=False,rev=0.0007,th=0.2,more=False):
        """
        probe for hole in axis direction with max_force, max travel in axis magnitude
        if hole missed, hunt in circles with increasing radius, checking force iter times each circle
        if R==True: rotate gripper with low power 
        """
        start_pose = self.getl()
        if R!=False:
            self.rotate_gripper_cont(var=R)
        self.force_move(axis,acc=0.1,vel=0.01,force=max_force)
        if R!=False:
            self.wait_for_gripper()
        end_pose = self.getl()
        dif = 0
        index = 0
        for i in range(0,3):
            if axis[i] != 0:
                travel_thresh = abs(axis[i])-0.003
                dif = end_pose[i]-start_pose[i]
                axis[i] -= dif
                index = i
        print(dif)
        if abs(dif)<travel_thresh:
            time.sleep(0.1)
            start_force = self.get_forces()[index]
            print("hunting start force: ",start_force)
            self.movel_tool([0,0,-rev,0,0,0])
            #have hit the board so hunt
            #time.sleep(0.3)
            #start_force = self.get_forces()[index]
            #print("hunting start force: ",start_force)
            if R!=False:
                self.rotate_gripper_cont(var=R)
            for j in range(1,circles+1):
                for i in range(0,iter):
                    if index == 0:
                        self.translatel([end_pose[0],end_pose[1]+j*hunting_radius*np.sin(i*2*pi/iter),end_pose[2]+j*hunting_radius*np.cos(i*2*pi/iter)],acc=0.05)
                    elif index == 1:
                        self.translatel([end_pose[0]+j*hunting_radius*np.sin(i*2*pi/iter),end_pose[1],end_pose[2]+j*hunting_radius*np.cos(i*2*pi/iter)],acc=0.05)
                    elif index == 2:
                        self.translatel([end_pose[0]+j*hunting_radius*np.sin(i*2*pi/iter),end_pose[1]+j*hunting_radius*np.cos(i*2*pi/iter),end_pose[2]],acc=0.05)
                    #print(self.get_forces()[index])
                    #print(self.get_forces()[index])
                    #print(self.get_forces()[index])
                    #print(self.get_forces()[index])
                    time.sleep(0.01)
                    force = self.get_forces()[index] 
                    print("measured force: ",force)
                    if (start_force>0 and force<th*start_force) or ((start_force<0 and force>th*start_force)):
                        if R!=False:
                            self.wait_for_gripper()
                        start_pose = self.getl()
                        self.force_move(axis,acc=0.1,vel=0.01,force=max_force)
                        stop_pose = self.getl()
                        if more == True:
                            end_pose[index] = stop_pose[index]
                            if abs(start_pose[index]-stop_pose[index])>0.002:
                                return True
                            else:
                                self.movel_tool([0,0,-rev,0,0,0])
                        else:
                            return True
            if R!=False:
                self.wait_for_gripper()
            self.translatel(end_pose)
            return False
        return True

    def screw_in(self,M,R=10,robot=False):
        """
        rotate gripper cw and move in tool z-axis by standard pitch M for each rotation detected
        """
        if M==0:
            dz=0.0
        elif M==1:
            dz=0.00025
        elif M==2:
            dz=0.0004
        elif M==3:
            dz=0.0005
        elif M==4:
            dz=0.0007
        elif M==5:
            dz=0.0008
        elif M==6:
            dz=0.001
        elif M==8:
            dz=0.00125
        elif M==10:
            dz=0.0015
        elif M==12:
            dz=0.00175
        else:
            dz = (M*1.25/9+0.25/3)/1000.0
        self.rotate_gripper_cw(var=R,wait=False)
        toc = time.time()
        while True:
            ipt = bytes.decode(self.ee.readline())
            print(ipt)
            tic = time.time()
            if ipt == "SWITCH TRIGGERED\r\n":
                if robot==False:
                    self.movel_tool([0,0,dz,0,0,0],min_time=tic-toc,wait=False)
                else:
                    burt.movel_tool([0,0,dz,0,0,0],min_time=tic-toc,wait=False)
                toc = time.time()
            if ipt == "done\r\n":
                return True
        return False

    def screw_out(self,M,R=10):
        """
        rotate gripper ccw and move in tool -ve z-axis by standard pitch M for each rotation detected
        """
        if M==0:
            dz=0.0
        elif M==1:
            dz=0.00025
        elif M==2:
            dz=0.0004
        elif M==3:
            dz=0.0005
        elif M==4:
            dz=0.0007
        elif M==5:
            dz=0.0008
        elif M==6:
            dz=0.001
        elif M==8:
            dz=0.00125
        elif M==10:
            dz=0.0015
        elif M==12:
            dz=0.00175
        else:
            dz = (M*1.25/9+0.25/3)/1000.0
        self.rotate_gripper_ccw(var=R,wait=False)
        toc = time.time()
        while True:
            ipt = bytes.decode(self.ee.readline())
            print(ipt)
            tic = time.time()
            if ipt == "SWITCH TRIGGERED\r\n":
                self.movel_tool([0,0,-dz,0,0,0],min_time=tic-toc,wait=False)
                toc = time.time()
            if ipt == "done\r\n":
                return True
        return False

    #-------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #
    #                                                                      Dual Robot Functions
    #
    #-------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #def calc_offset(self):  # assume x,y and rz offset only (z, rx and ry = 0)
    #    """
    #    calculate the x,y and rotation offset between the two robots using the calibration position
    #    """
    #    x = scipy.optimize.least_squares(self.F,np.array([0,0,0]))
    #    self.offset = x['x']
    #    return

    #def F(self,x):
    #    return np.array([x[0]+wp.dual_rob_cal[0][0]-wp.dual_rob_cal[3][0]*np.cos(x[2])-wp.dual_rob_cal[3][1]*np.sin(x[2]),
    #            x[1]+wp.dual_rob_cal[0][1]+wp.dual_rob_cal[3][0]*np.sin(x[2])-wp.dual_rob_cal[3][1]*np.cos(x[2]),
    #            x[0]+wp.dual_rob_cal[1][0]-wp.dual_rob_cal[4][0]*np.cos(x[2])-wp.dual_rob_cal[4][1]*np.sin(x[2]),
    #            x[1]+wp.dual_rob_cal[1][1]+wp.dual_rob_cal[4][0]*np.sin(x[2])-wp.dual_rob_cal[4][1]*np.cos(x[2])])



    #def calc_pose(self,burt_pose):
    #    """
    #    returns the burt pose in urnie space
    #    """
    #    self.target_pose = burt_pose
    #    x = scipy.optimize.least_squares(self.F2,np.array([0,0]))
    #    return [x['x'][0],x['x'][1],burt_pose[2],burt_pose[3],burt_pose[4],burt_pose[5]]

    def calc_pose(self,burt_pose):
        return self.burt_to_urnie(burt_pose)

    #def F2(self,x):
    #    return np.array([x[0]*np.cos(self.offset[2])+x[1]*np.sin(self.offset[2])-self.offset[0]-self.target_pose[0],
    #                     -x[0]*np.sin(self.offset[2])+x[1]*np.cos(self.offset[2])-self.offset[1]-self.target_pose[1]])

    #def calc_inverse(self,urnie_pose):
    #    """
    #    returns the x,y for burt to reach the urnie pose
    #    """
    #    self.target_pose = urnie_pose
    #    x = -self.offset[0] + self.target_pose[0]*np.cos(self.offset[2]) + self.target_pose[1]*np.sin(self.offset[2])
    #    y = -self.offset[1] - self.target_pose[0]*np.sin(self.offset[2]) + self.target_pose[1]*np.cos(self.offset[2])
    #    return [x,y,urnie_pose[2],urnie_pose[3],urnie_pose[4],urnie_pose[5]]

    def calc_inverse(self,urnie_pose):
        return self.urnie_to_burt(urnie_pose)

    def burt_to_urnie(self,burt_pose):
        """
        returns the burt pose in urnie space
        """
        gamma = (((burt_pose[1]         - wp.dual_rob_cal[0][1]) * (wp.dual_rob_cal[2][0]  -wp.dual_rob_cal[0][0]) - (burt_pose[0]          - wp.dual_rob_cal[0][0]) * (wp.dual_rob_cal[2][1] - wp.dual_rob_cal[0][1]))/
                ((wp.dual_rob_cal[1][1] - wp.dual_rob_cal[0][1]) * (wp.dual_rob_cal[2][0] - wp.dual_rob_cal[0][0]) - (wp.dual_rob_cal[1][0] - wp.dual_rob_cal[0][0]) * (wp.dual_rob_cal[2][1] - wp.dual_rob_cal[0][1])))
        lambd = (burt_pose[0]-wp.dual_rob_cal[0][0]-gamma*(wp.dual_rob_cal[1][0]-wp.dual_rob_cal[0][0]))/(wp.dual_rob_cal[2][0]-wp.dual_rob_cal[0][0])
        x = wp.dual_rob_cal[3][0]+lambd*(wp.dual_rob_cal[5][0]-wp.dual_rob_cal[3][0])+gamma*(wp.dual_rob_cal[4][0]-wp.dual_rob_cal[3][0])
        y = wp.dual_rob_cal[3][1]+lambd*(wp.dual_rob_cal[5][1]-wp.dual_rob_cal[3][1])+gamma*(wp.dual_rob_cal[4][1]-wp.dual_rob_cal[3][1])
        z = burt_pose[2]         +lambd*((wp.dual_rob_cal[5][2]-wp.dual_rob_cal[3][2])-(wp.dual_rob_cal[2][2]-wp.dual_rob_cal[0][2]))+gamma*((wp.dual_rob_cal[4][2]-wp.dual_rob_cal[3][2])-(wp.dual_rob_cal[1][2]-wp.dual_rob_cal[0][2]))
        return [x,y,z,burt_pose[3],burt_pose[4],burt_pose[5]] 

    def urnie_to_burt(self,urnie_pose):
        """
        returns the x,y for burt to reach the urnie pose
        """
        gamma = (((urnie_pose[1]        - wp.dual_rob_cal[3][1]) * (wp.dual_rob_cal[5][0] - wp.dual_rob_cal[3][0]) - (urnie_pose[0]         - wp.dual_rob_cal[3][0]) * (wp.dual_rob_cal[5][1] - wp.dual_rob_cal[3][1]))/
                ((wp.dual_rob_cal[4][1] - wp.dual_rob_cal[3][1]) * (wp.dual_rob_cal[5][0] - wp.dual_rob_cal[3][0]) - (wp.dual_rob_cal[4][0] - wp.dual_rob_cal[3][0]) * (wp.dual_rob_cal[5][1] - wp.dual_rob_cal[3][1])))
        lambd = (urnie_pose[0]-wp.dual_rob_cal[3][0]-gamma*(wp.dual_rob_cal[4][0]-wp.dual_rob_cal[3][0]))/(wp.dual_rob_cal[5][0]-wp.dual_rob_cal[3][0])
        x = wp.dual_rob_cal[0][0]+lambd*(wp.dual_rob_cal[2][0]-wp.dual_rob_cal[0][0])+gamma*(wp.dual_rob_cal[1][0]-wp.dual_rob_cal[0][0])
        y = wp.dual_rob_cal[0][1]+lambd*(wp.dual_rob_cal[2][1]-wp.dual_rob_cal[0][1])+gamma*(wp.dual_rob_cal[1][1]-wp.dual_rob_cal[0][1])
        z = urnie_pose[2]        +lambd*((wp.dual_rob_cal[2][2]-wp.dual_rob_cal[0][2])-(wp.dual_rob_cal[5][2]-wp.dual_rob_cal[3][2]))+gamma*((wp.dual_rob_cal[1][2]-wp.dual_rob_cal[0][2])-(wp.dual_rob_cal[4][2]-wp.dual_rob_cal[3][2]))
        return [x,y,z,urnie_pose[3],urnie_pose[4],urnie_pose[5]] 

    def calc_pedestal(self,pedestal_pose,robot=None):
        """
        returns the pose to reach the pedestal pose. urnie contains pedestal calibration
        """
        if self.side == 'right':
            return self.pedestal.calc_pose(pedestal_pose)
        else:
            return self.urnie_to_burt(robot.pedestal.calc_pose(pedestal_pose))

    def movel_pedestal(self,pedestal_pose,robot=None, acc=0.5, vel=0.5, min_time=0, radius=0, wait=True):
        """
        linear move to pedestal pose
        """
        if self.side == 'right':
            return self.movel(self.pedestal.calc_pose(pedestal_pose), acc=acc, vel=vel, min_time=min_time, radius=radius, wait=wait)
        else:
            return self.movel(self.urnie_to_burt(robot.pedestal.calc_pose(pedestal_pose)), acc=acc, vel=vel, min_time=min_time, radius=radius, wait=wait)

    def movejl_pedestal(self,pedestal_pose,robot=None, acc=0.5, vel=0.5, min_time=0, radius=0, wait=True):
        """
        joint move in linear space to pedestal pose
        """
        if self.side == 'right':
            return self.movejl(self.pedestal.calc_pose(pedestal_pose), acc=acc, vel=vel, min_time=min_time, radius=radius, wait=wait)
        else:
            return self.movejl(self.urnie_to_burt(robot.pedestal.calc_pose(pedestal_pose)), acc=acc, vel=vel, min_time=min_time, radius=radius, wait=wait)

    def translatel_pedestal(self,pedestal_pose,robot=None, acc=0.5, vel=0.5, min_time=0, radius=0, wait=True):
        """
        translate to pedestal pose with linear move
        """
        if self.side == 'right':
            return self.translatel(self.pedestal.calc_pose([pedestal_pose[0],pedestal_pose[1],pedestal_pose[2],0,pi,0]), acc=acc, vel=vel, min_time=min_time, radius=radius, wait=wait)
        else:
            return self.translatel(self.urnie_to_burt(robot.pedestal.calc_pose([pedestal_pose[0],pedestal_pose[1],pedestal_pose[2],0,pi,0])), acc=acc, vel=vel, min_time=min_time, radius=radius, wait=wait)

    def translatejl_pedestal(self,pedestal_pose,robot=None, acc=0.5, vel=0.5, min_time=0, radius=0, wait=True):
        """
        translate to pedestal pose wiht joint move in linear space
        """
        if self.side == 'right':
            return self.translatejl(self.pedestal.calc_pose([pedestal_pose[0],pedestal_pose[1],pedestal_pose[2],0,pi,0]), acc=acc, vel=vel, min_time=min_time, radius=radius, wait=wait)
        else:
            return self.translatejl(self.urnie_to_burt(robot.pedestal.calc_pose([pedestal_pose[0],pedestal_pose[1],pedestal_pose[2],0,pi,0])), acc=acc, vel=vel, min_time=min_time, radius=radius, wait=wait)








class kg_robot_dashboard():
    def __init__(self, host):
        self.open=False
        try:
            self.c = socket.create_connection((host, 29999), timeout=1)
            time.sleep(2)
        except socket.error as socketerror:
            print("problem connecting to the socket")
            self.reconnect(host)

        if self.open == False:
            try:
                print(bytes.decode(self.c.recv(1024)))
                self.open=True
            except socket.error as socketerror:
                print("problem reading from the socket")
                self.c.close()
                time.sleep(1)
                self.reconnect(host)

    def init(self):
        print(self.socket_send("PolyscopeVersion\n"))
        if self.socket_send("robotmode\n")!="Robotmode: RUNNING\n":
            print(self.socket_send("power on\n"))
            print(self.socket_send("brake release\n"))
        print(self.socket_send("load kg_client.urp\n"))
        print(self.socket_send("stop\n"))
        while self.socket_send("robotmode\n")!="Robotmode: RUNNING\n":
            time.sleep(0.5)
        print(self.socket_send("play\n"))
        #print(self.socket_send("quit\n"))
        self.c.close()
        self.open=False

    def reconnect(self,host):
        print("attempting to reconnect...")
        if self.open==False:
            try:
                time.sleep(1)
                self.c = socket.create_connection((host, 29999), timeout=0.5)
                time.sleep(1)
            except socket.error as socketerror:
                self.reconnect(host)

            if self.open == False:
                try:
                    print(bytes.decode(self.c.recv(1024)))
                    self.open=True
                except socket.error as socketerror:
                    print("problem reading from the socket")
                    self.c.close()
                    time.sleep(1)
                    self.reconnect(host)
        
    def socket_send(self,prog):
        msg = "No message from robot"
        try:
            self.c.send(str.encode(prog))
            # Wait for reply
            msg=bytes.decode(self.c.recv(1024))

        except socket.error as socketerror:
            print("........................Dashboard error :(.........................")
        return msg