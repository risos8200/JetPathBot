#!/usr/bin/env python
import rospy
import time

from Adafruit_MotorHAT import Adafruit_MotorHAT
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy
import math
import numpy as np

# initialization
if __name__ == '__main__':

        # setup ros node
        rospy.init_node('jetbot_test')
        ctr_pub = rospy.Publisher('/ctrl_cmd', Float32MultiArray, queue_size=1)

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
        def forward(t):
            speed_l = 0.822
            speed_r = 0.789
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
            speed_l = 0.822
            speed_r = -0.789
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
            speed_l = -0.822
            speed_r = 0.789
            for i in range(t):
                msg = Float32MultiArray()
                msg.data = [move, speed_l, speed_r]
                ctr_pub.publish(msg)
                time.sleep(0.1)

            msg.data = [stop, speed_l, speed_r]
            ctr_pub.publish(msg)
            time.sleep(1.0)
        
        #reading waypoints file
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
        
        #to find angles between two points
        def angle(x1,y1,x2,y2):
            change_x=x2-x1
            change_y=y2-y1
            return(math.atan2(change_y,change_x))
        
        
        #to find distance between two points
        def dist(x1,y1,x2,y2):
            return(math.sqrt( (x2 - x1)**2 + (y2 - y1)**2 ))
        
        #telling robot to turn
        def turn(x,world_angle):
            if x>0:
                y=int(abs(x/3.14)*8.5)
                print('y',y)
                if(y>0.1):
                    right(y)
                    world_angle=round(world_angle+abs(x),2)
            else:
                y=int(abs(x/3.14)*7.7)
                print('y',y)
                if(y>0.1):
                    left(y)
                    world_angle=round(world_angle-abs(x),2)
            return(world_angle)
        
        #calculation of the path
        def cal_path(coord,current,world_angle):
                desired_angle=angle(current[0],current[1],coord[0],coord[1])
                to_move=round(desired_angle,2)
                to_move=round(to_move-world_angle,2)
                
                # if to_move>3.15:
                #     to_move=3.14-to_move                
                # elif to_move<-3.15:
                #     to_move=-(to_move+3.14)
                print('to_move',to_move)
                world_angle=turn(to_move,world_angle) #telling robot to move turn a certain angle

                distance=dist(current[0],current[1],coord[0],coord[1])
                forward(int(distance*34)) #telling robot to move forward with a specific amount of distance
                print(distance)

                get_current=round(coord[2]-world_angle,2)
                # print('get_current2',get_current)
                # if get_current>3.15:
                #     get_current=3.14-get_current
                # elif get_current<-3.15:
                #     get_current=-(get_current+3.14)
                print('get_current',get_current)
                world_angle=turn(get_current,world_angle)
                print('world_angle',world_angle)

                current=coord
                return(current,world_angle)


        current=path[0]
        path.pop(0)
        world_angle=0 #keeping the current angle orientation of the robot in real world
        # control of sending in the waypoints
        for i in path:
            print(i)
            current,world_angle=cal_path(i,current,world_angle)
        