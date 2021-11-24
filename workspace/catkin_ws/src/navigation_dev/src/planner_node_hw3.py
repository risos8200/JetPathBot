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
# from filterpy.kalman import ExtendedKalmanFilter as EKF
from numpy import array, sqrt
from math import sqrt, tan, cos, sin, atan2
import sympy
from numpy.linalg import inv
from sympy import Matrix
from numpy.linalg import multi_dot



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

#extended kalman filter implementation
class ekf:
    def __init__(self, dt):
        self.dt = dt
        # a, x, y, v, w, theta, time = symbols('a, x, y, v, w, theta, t')
        # d = v*time
        # beta = (d/w)*sympy.tan(a)
        # r = w/sympy.tan(a)
    
        # self.fxu = Matrix([[x-r*sympy.sin(theta)+r*sympy.sin(theta+beta)],[y+r*sympy.cos(theta)-r*sympy.cos(theta+beta)],[theta+beta]])

        # self.F_j = self.fxu.jacobian(Matrix([x, y, theta]))
        # self.V_j = self.fxu.jacobian(Matrix([v, a]))

        self.loc=[0,0,0]
        self.corr=np.zeros((3,3))
        self.Q=np.eye(3)
        self.Q[0,0]=0.25
        self.Q[1,1]=0.25
        self.Q[3,3]=0.55
        self.R=np.eye(2)
        self.R[0,0]=0.02
        self.R[1,1]=0.07
        self.flist=[]
        # self.v, self.a, self.theta = v, a, theta

    # st|t1 = f (st1|t1,ut1,0), predicition step 1(ppt)
    def update_pos(self,v,w,dt):
        self.loc[0]+=v*cos(w*dt)
        self.loc[1]+=v*sin(w*dt)
        self.loc[2]+=w*dt

    #predicition step 1(ppt) t|t1 = Ft t1|t1F Tt + Wt Qt1W T
    def update_cov(self,v,w,dt):
        x=self.loc[0]
        y=self.loc[1]
        theta=self.loc[2]
        r=v/w
        Gx= Matrix([[x-r*sympy.sin(theta)+r*sympy.sin(theta+w*dt)],[y+r*sympy.cos(theta)-r*sympy.cos(theta+w*dt)],[theta+w*dt]])
        G=Gx.jacobian(Matrix([x, y, theta]))
        G=array(self.G.evalf()).astype(float)
        self.corr=multi_dot([G,self.corr, G.T]) + self.Q
        file1 = open("covarance.txt", "a") 
        file1.write(array(self.corr).astype(float))
        file1.close()
    
    # to get zt values
    def zval(self,i):
        x, y = self.loc[0], self.loc[1]
        d = np.sqrt((i[0] - x)**2 + (i[1] - y)**2)  
        a = atan2(i[1] - y, i[0] - x) - i[2]
        z = np.array([[d],[a]])
        return z
    
    #H jacobian
    def H_J(self, i):
        x, y = self.loc[0], self.loc[1]
        x_i= i[0]
        y_i = i[1]
        hyp = (x_i - x[0, 0])**2 + (y_i - x[1, 0])**2
        dist = sqrt(hyp)

        H = array([[-(x_i - x[0, 0]) / dist, -(y_i - x[1, 0]) / dist, 0],[ (y_i - x[1, 0]) / hyp,  -(x_i - x[0, 0]) / hyp, -1]])
        return H

    #to get h(st|t1,0)
    def Hof(self, i):
        x, y = self.loc[0], self.loc[1]
        px = i[0]
        py = i[1]
        dist = sqrt((px - x[0, 0])**2 + (py - x[1, 0])**2)
        Hof = array([[dist],[atan2(py - x[1, 0], px - x[0, 0]) - x[2, 0]]])
        return Hof
    
    #to check threshold between landmarks
    def threshold_check(self,new):
        if(self.flist==[]):return(False)
        for j in self.flist:
            for i in new:
                if(i[0]==j[0]):
                    # if(abs(i[1]-j[1])<0.3) and (abs(abs(i[3])-abs(j[3]))<0.34):
                    if (abs(abs(i[3])-abs(j[3]))<0.34) or (abs(abs(i[2])-abs(j[2]))<0.34):
                        return(True)
                    else:
                        return(False)
                else:
                    return(False)


    #Measurement update computation
    def correct(self,f):
        for i in f:
            check=self.threshold_check(i,f)
            if check:
                z=ekf.zval(i)
                h_j=ekf.H_K(i)
                h_jt=h_j.transpose()
                hof=ekf.Hof(i)
                k=multi_dot([self.corr,h_jt]) * inv(multi_dot([h_j, self.corr,h_jt]) + self.R)
                self.loc=self.loc + k*(z-hof)
                self.corr=np.identity(len(k)-1)- multi_dot([multi_dot([k, h_j]), self.corr])
            else:
                self.flist.append(i)
                self.loc.append(i[0],i[1])
                np.pad(self.corr, ((0,2),(0,2)), mode='constant', constant_values=0)
                self.corr[len(self.corr)-2,len(self.corr)-2]=self.R[0,0]
                self.corr[len(self.corr)-1,len(self.corr)-1]=self.R[1,1]



#motion planner
class motion:
    def __init__(self):
        # self.end_motion=False
        # self.reached=False
        self.f=[]      
        # self.current_xy=[0,0]
        # self.current_angle=0
        self.ctr_pub = rospy.Publisher('/ctrl_cmd', Float32MultiArray, queue_size=1)
        # self.counter=0
        # self.cur_wt = 0
        # self.waypoints = waypoints
        # self.step = 0.10
        self.dt=1
        self.control_obj = control()
        self.ekf_obj=ekf(self.dt)
        self.dt=1
        self.mover()
    
    #to get data from april tags data
    def pose_callback(self,msg):
        f_i=[]
        ids=[]
        for apriltag_id, pose in zip(msg.pose.id, msg.matrix):
            for i in range(0,len(apriltag_id)):
                if(apriltag_id[i] not in ids):
                    pose_i=np.asarray(list(pose[i].matrix)).reshape((4,4))
                    pos_matrix=np.asarray(pose_i).reshape((3,4))
                    translation=pos_matrix[:3,3]
                    rotation=pos_matrix[:3,:3]
                    x=round(translation[2],1)
                    y=-round(translation[0],1)
                    r=np.sqrt(x**2+y**2)
                    theta=math.atan(y/x)
                    current_angle=round(rotat_to_euler(rotation)[1],2)
                    f_i.append([f_i,r,theta,current_angle])
                    ids.append(apriltag_id)
        self.f=f_i



            # pos_matrix=np.asarray(msg.pose.matrix).reshape((3,4))
            # translation=pos_matrix[:3,3]
            # rotation=pos_matrix[:3,:3]
            # print(translation)
            # print(rotat_to_euler(rotation)[1])
            # self.current_xy[0]=round(translation[2],1)
            # self.current_xy[1]=-round(translation[0],1)
            # self.current_angle=round(rotat_to_euler(rotation)[1],2)
            # if(self.reached==False) and (self.counter!=1):
            #     self.counter+=1
            # elif(self.reached==False) and (self.counter==1):
            #     self.mover()
            #     self.counter=0
            # elif(self.reached==True) and (self.wavepoint_reached==False):
            #     self.rotate_point()
            # elif(self.reached==True) and (self.wavepoint_reached==True):
            #     self.control_obj.stop_motion()
     
    #circle motion(additional step for 8 motion)
    def mover(self):
        i=1
        while(i<=24):
            # curren_xy=self.current_xy
            # curren_angle=self.current_angle
            # goal_pose = self.waypoints[self.cur_wt]
            # final_pos = goal_pose[:2]
            # final_orien = goal_pose[2]
            # desired_angle=angle(curren_xy[0],curren_xy[1],final_pos[0],final_pos[1])
            # to_move=round(desired_angle,2)
            # to_move=round(to_move-curren_angle,2)
            # distance=dist(curren_xy[0],curren_xy[1],final_pos[0],final_pos[1])
            distance=0.2
            # print('distance',distance, 'desired xy',final_pos)
            to_move=0.78
            # to_move = angle(final_pos[1] - curren_xy[1], final_pos[0] - curren_xy[0]) - curren_angle
            # to_move = abs(to_move) % (math.pi * 2) * to_move / (abs(theta + 10**-10))
            # if(abs(distance)>0.05):
            v, t1 = cal_v(distance * 0.10, self.dt, -1)
            o, t2 = cal_w(to_move * max(distance, 0.50) * 0.3, self.dt)
            rospy.Subscriber("/current_pose", Pose, self.pose_callback)  
            self.control_obj.wv_m(v, -o * 2, (t1 + t2) / 2)
            self.ekf_obj.update_pos(v,o,self.dt)
            self.ekf_obj.update_cov(v,o,self.dt)
            self.ekf_obj.correct(self.f)
            i+=1

        # # additional step for 8 motion
        # while(i<=24):
        #     distance=0.16
        #     to_move=-0.78
        #     v, t1 = cal_v(distance * 0.10, self.step, -1)
        #     o, t2 = cal_w(to_move * max(distance, 0.50) * 0.3, self.step)
        #     self.control_obj.wv_m(v, -o * 2, (t1 + t2) / 2)
        #     self.ekf_obj.update_pos(v,o,self.dt)
        #     self.ekf_obj.update_cov(v,o,self.dt)
        #     self.ekf_obj.correct(self.f)
        #     i+=1
    

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
            print('rotation',thetadiff, omeganow, timenow)
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
    # download_thread = threading.Thread())
    # rospy.Subscriber("/current_pose", Pose, pose_callback)
    # path=[]
    # with open('waypoints2.txt') as f:
    #     lines = f.readlines()
    # for i in lines:
    #     i=i.strip()
    #     l1=i.split(',')
    #     l2=[]
    #     for j in l1:
    #         l2.append(float(j))
    #     path.append(l2)

    # time.sleep(1)
    # while(i<24):
    motion()
    rospy.spin()
