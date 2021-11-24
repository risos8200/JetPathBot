#!/usr/bin/env python

import rospy
import cv2
import apriltag
from navigation_dev.msg import AprilDetections
from navigation_dev.msg import Pose 
from std_msgs.msg import Float32MultiArray
import math
import numpy as np
import time


#old code reverse, forward, right, left, turn not used 
move = 0.0
stop = 1.0


# reverse 
def reverse(t):
    speed_l = -0.822
    speed_r = -0.789
    for i in range(t):
        msg = Float32MultiArray()
        msg.data = [move, speed_l, speed_r]
        ctr_pub.publish(msg)
        time.sleep(0.1)

    msg.data = [stop, speed_l, speed_r]
    ctr_pub.publish(msg)
    time.sleep(1.0)

# forward
def forward(t,ctr_pub):
    speed_l = 0.822
    speed_r = 0.789
    for i in range(t):
        msg = Float32MultiArray()
        msg.data = [move, speed_l, speed_r]
        ctr_pub.publish(msg)
        time.sleep(0.1)

    msg.data = [stop, speed_l, speed_r]
    ctr_pub.publish(msg)
    # time.sleep(1.0)

# right
def right(t,ctr_pub):
    speed_l = 0.822
    speed_r = -0.789
    for i in range(t):
        msg = Float32MultiArray()
        msg.data = [move, speed_l, speed_r]
        ctr_pub.publish(msg)
        time.sleep(0.1)

    msg.data = [stop, speed_l, speed_r]
    ctr_pub.publish(msg)


# left
def left(t,ctr_pub):
    speed_l = -0.822
    speed_r = 0.789
    for i in range(t):
        msg = Float32MultiArray()
        msg.data = [move, speed_l, speed_r]
        ctr_pub.publish(msg)
        time.sleep(0.1)

    msg.data = [stop, speed_l, speed_r]
    ctr_pub.publish(msg)

#telling robot to turn
def turn(x,clt):
    if x>0:
        y=int(abs(x/3.14)*8.5)
        print('y',y)
        if(y>0.1):
            right(y,clt)
    else:
        y=int(abs(x/3.14)*7.7)
        print('y',y)
        if(y>0.1):
            left(y,clt)

def angle(x1,y1,x2,y2):
        change_x=x2-x1
        change_y=y2-y1
        return(math.atan2(change_y,change_x))
#old code end

#function to convert rotation matrix to euler
def rotat_to_euler(R):
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    return np.array([x, y, z])


#To control motor
class control:

    def __init__(self):
        self.pub = rospy.Publisher('/ctrl_cmd', Float32MultiArray, queue_size=1)
        self.wheel_params = {'R': 0.03, 'L': 0.15, 'pos': {'slope': 0.3245 * 100, 'intercept': -0.1999 * 100}, 'neg': {'slope': 0.3979 * 100, 'intercept': 0.2004 * 100}}
        self.l_to_r = 0.88
        self.move = 0.0
        self.stop = 1.0
        self.step = 0.08


    def p_m(self, p):
        direction = 0
        if p > 0.5:
            direction = 1
            return (p - self.wheel_params['pos']['intercept']) / self.wheel_params['pos']['slope']
        if p < -0.5:
            direction = -1
            return (p - self.wheel_params['neg']['intercept']) / self.wheel_params['neg']['slope']
        else:
            return 0

    def wv_m(self, v, w, d):
        p_r = w * self.wheel_params['L'] / (2 * self.wheel_params['R']) + v / self.wheel_params['R']
        p_l = -w * self.wheel_params['L'] / (2 * self.wheel_params['R']) + v / self.wheel_params['R']
        motor_l, motor_r = self.p_m(p_l), self.p_m(p_r)
        step = int(d / self.step)
        total_t= 0.0
        while total_t <= d:
            msg = Float32MultiArray()
            msg.data = [self.move, -motor_l * self.l_to_r, -motor_r]
            self.pub.publish(msg)
            time.sleep(self.step)
            total_t += self.step
        msg = Float32MultiArray()
        msg.data = [self.stop, -0.45, -0.45]
        self.pub.publish(msg)

    def stop_motion(self):
        msg = Float32MultiArray()
        msg.data = [self.stop, 0, 0]
        self.pub.publish(msg)


def cal_w(a, step):
    o = a / step
    o_dir = o / (abs(o + 10**-10))
    if abs(o) > math.pi / 3:
        return math.pi / 3 * o_dir, step

    return o, step

def cal_v(d, step, dir):
    v = d / step
    if v > 0.25:
        return 0.25 * dir, step
    return v * dir, step

def turn(x,clt):
            if x>0:
                y=int(abs(x/3.14)*8.5)
                print('y',y)
                if(y>0.1):
                    right(y,clt)
            else:
                y=int(abs(x/3.14)*7.7)
                print('y',y)
                if(y>0.1):
                    left(y,clt)
    

def dist(x1,y1,x2,y2):
    return(math.sqrt( (x2 - x1)**2 + (y2 - y1)**2 ))


#motion planner
class motion:
    def __init__(self,waypoints):
        self.end_motion=False
        self.reached=False
        self.sub = rospy.Subscriber("/current_pose", Pose, self.pose_callback)
        self.current_xy=[0,0]
        self.current_angle=0
        self.ctr_pub = rospy.Publisher('/ctrl_cmd', Float32MultiArray, queue_size=1)
        self.counter=0
        self.cur_wt = 0
        self.waypoints = waypoints
        self.step = 0.10
        self.control_obj = control()
    
    #to get data from april tags
    def pose_callback(self,msg):
            pos_matrix=np.asarray(msg.pose.matrix).reshape((3,4))
            translation=pos_matrix[:3,3]
            rotation=pos_matrix[:3,:3]
            print(translation)
            print(rotat_to_euler(rotation)[1])
            self.current_xy[0]=round(translation[2],1)
            self.current_xy[1]=-round(translation[0],1)
            self.current_angle=round(rotat_to_euler(rotation)[1],2)
            if(self.reached==False) and (self.counter!=1):
                self.counter+=1
            elif(self.reached==False) and (self.counter==1):
                self.mover()
                self.counter=0
            elif(self.reached==True) and (self.wavepoint_reached==False):
                self.rotate_point()
            elif(self.reached==True) and (self.wavepoint_reached==True):
                self.control_obj.stop_motion()
     
    def mover(self):
            curren_xy=self.current_xy
            curren_angle=self.current_angle
            goal_pose = self.waypoints[self.cur_wt]
            final_pos = goal_pose[:2]
            final_orien = goal_pose[2]
            desired_angle=angle(curren_xy[0],curren_xy[1],final_pos[0],final_pos[1])
            # to_move=round(desired_angle,2)
            # to_move=round(to_move-curren_angle,2)
            distance=dist(curren_xy[0],curren_xy[1],final_pos[0],final_pos[1])
            print('distance',distance, 'desired xy',final_pos)
            to_move = angle(final_pos[1] - curren_xy[1], final_pos[0] - curren_xy[0]) - curren_angle
            to_move = abs(to_move) % (math.pi * 2) * to_move / (abs(theta + 10**-10))
            if(abs(distance)>0.05):
                v, t1 = cal_v(distance * 0.10, self.step, -1)
                o, t2 = cal_w(to_move * max(distance, 0.50) * 0.3, self.step)
                self.control_obj.wv_m(v, -o * 2, (t1 + t2) / 2)
            else:
                print(self.reached)
                self.reached=True
    

    def rotate_point(self):
            curren_angle = self.current_angle
            self.control_obj.stop_motion()
            self.reached = True
            way_next = self.waypoints[self.cur_wt]
            dif = way_next[2]-curren_angle
            
            o = - np.pi * 3 / 2 * 1
            t = abs(dif) / abs(i)
            if nextwp[2] == 0:
                omeganow = 0.0
            print('roatation',thetadiff, omeganow, timenow)
            self.control_obj.wv_m(0, -omeganow, timenow + additional_startup_time)
            self.cur_wt += 1
            self.reached=False


    
        # old path calulation
        # ctrl_pub.publish(cmd_msg)
        #to find angles between two points
        #calculation of the path
        # def cal_path(coord,current,world_angle):
        #         desired_angle=angle(current[0],current[1],coord[0],coord[1])
        #         to_move=round(desired_angle,2)
        #         to_move=round(to_move-world_angle,2)
                
        #         # if to_move>3.15:
        #         #     to_move=3.14-to_move                
        #         # elif to_move<-3.15:
        #         #     to_move=-(to_move+3.14)
        #         print('to_move',to_move)
        #         world_angle=turn(to_move,world_angle) #telling robot to move turn a certain angle

        #         distance=dist(current[0],current[1],coord[0],coord[1])
        #         forward(int(distance*34)) #telling robot to move forward with a specific amount of distance
        #         print(distance)

        #         get_current=round(coord[2]-world_angle,2)
        #         # print('get_current2',get_current)
        #         # if get_current>3.15:
        #         #     get_current=3.14-get_current
        #         # elif get_current<-3.15:
        #         #     get_current=-(get_current+3.14)
        #         print('get_current',get_current)
        #         world_angle=turn(get_current,world_angle)
        #         print('world_angle',world_angle)

        #         current=coord
        #         return(current,world_angle)


        # current=path[0]
        # path.pop(0)
        # world_angle=0 #keeping the current angle orientation of the robot in real world
        # # control of sending in the waypoints
        # for i in path:
        #     print(i)
        #     current,world_angle=cal_path(i,current,world_angle)


if __name__ == "__main__":
    rospy.init_node('planner_node')
    # rospy.Subscriber("/current_pose", Pose, pose_callback)
    path=[]
    with open('waypoints2.txt') as f:
        lines = f.readlines()
    for i in lines:
        i=i.strip()
        l1=i.split(',')
        l2=[]
        for j in l1:
            l2.append(float(j))
        path.append(l2)
    motion(path)
    time.sleep(1)
    rospy.spin()
