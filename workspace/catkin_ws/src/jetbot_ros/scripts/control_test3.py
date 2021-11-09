#!/usr/bin/env python
import rospy
import time

from Adafruit_MotorHAT import Adafruit_MotorHAT
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy
import math


# initialization
if __name__ == '__main__':

        # setup ros node
        rospy.init_node('jetbot_test')
        ctr_pub = rospy.Publisher('/ctrl_cmd', Float32MultiArray, queue_size=1)

        move = 0.0
        stop = 1.0


        # reverse 
        def reverse(t):
            speed_l = -0.8
            speed_r = -0.8
            for i in range(t):
                msg = Float32MultiArray()
                msg.data = [move, speed_l, speed_r]
                ctr_pub.publish(msg)
                time.sleep(0.1)

            msg.data = [stop, speed_l, speed_r]
            ctr_pub.publish(msg)
            time.sleep(1.0)

        # forward
        def forward(t):
            speed_l = 0.8
            speed_r = 0.8
            for i in range(t):
                msg = Float32MultiArray()
                msg.data = [move, speed_l, speed_r]
                ctr_pub.publish(msg)
                time.sleep(0.1)

            msg.data = [stop, speed_l, speed_r]
            ctr_pub.publish(msg)
            time.sleep(1.0)

        # right
        def right(t):
            speed_l = 0.8
            speed_r = -0.8
            for i in range(t):
                msg = Float32MultiArray()
                msg.data = [move, speed_l, speed_r]
                ctr_pub.publish(msg)
                time.sleep(0.1)

            msg.data = [stop, speed_l, speed_r]
            ctr_pub.publish(msg)
            time.sleep(1.0)
        
        # left
        def left(t):
            speed_l = -0.8
            speed_r = 0.8
            for i in range(t):
                msg = Float32MultiArray()
                msg.data = [move, speed_l, speed_r]
                ctr_pub.publish(msg)
                time.sleep(0.1)

            msg.data = [stop, speed_l, speed_r]
            ctr_pub.publish(msg)
            time.sleep(1.0)
        
        path=[]
        with open('waypoints.txt') as f:
            lines = f.readlines()
        for i in lines:
            i=i.strip()
            l1=i.split(',')
            l2=[]
            for j in l1:
                l2.append(float(j))
            path.append(l2)
        
        def angle(x1,y1,x2,y2):
            change_x=x2-x1
            change_y=y2-y1
            return(math.atan2(change_y,change_x))
        
        def dist(x1,y1,x2,y2):
            return(math.sqrt( (x2 - x1)**2 + (y2 - y1)**2 ))
        
        def turn(x):
            if x<0:
                y=int(abs(x/3.14)*9)
                print('y',y)
                if(y!=0 and y>0.1):
                    left(int(abs((x/3.14)*9)))
            else:
                
                y=int(abs(x/3.14)*9)
                print('y',y)
                if(y!=0 and y>0.1):
                    right(int(abs(x/3.14)*9))
        def cal_path(coord,current):
                desired_angle=angle(current[0],current[1],coord[0],coord[1])
                # print('desiredangle',desired_angle)
                to_move=round(desired_angle,2)
                # print(to_move)
                to_move=round(to_move-current[2],2)
                # print(to_move)
                
                if to_move>3.15:
                    to_move=3.14-to_move
                
                elif to_move<-3.15:
                    to_move=-(to_move+3.14)
                print(to_move)
                # turn(to_move)
                distance=dist(current[0],current[1],coord[0],coord[1])
                # forward(int(distance*34))
                print(distance)
                get_current=round(coord[2]-to_move,2)
                # print('get_current2',get_current)
                if get_current>3.15:
                    get_current=3.14-get_current
                elif get_current<-3.15:
                    get_current=-(get_current+3.14)
                print('get_current',get_current)
                # turn(get_current)
                current=coord
                return(current)

        current=path[0]
        path.pop(0)


        
        for i in path:
            # print('current',current)
            # print('i',i)
            print(i)
            current=cal_path(i,current)
        