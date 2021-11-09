#!/usr/bin/env python

import rospy
import cv2
import apriltag
from std_msgs.msg import Float32MultiArray
from navigation_dev.msg import AprilDetections
from navigation_dev.msg import Pose
import math
import time
import math
import numpy as np

np.set_printoptions(precision=3)


class MotorController:

    def __init__(self):
        self.pub = rospy.Publisher(
            '/ctrl_cmd', Float32MultiArray, queue_size=1)
        # self.veh_params = {'R' : 0.031, 'L' : 0.150, 'wheel_pos' : {'slope' : 0.410013*100, 'intercept' : -0.215093*100}, 'wheel_neg' : {'slope' : 0.410013*100, 'intercept' : 0.215000*100}}
        # self.veh_params = {'R' : 0.031, 'L' : 0.150, 'wheel_pos' : {'slope' : 0.430013*100, 'intercept' : -0.215093*100}, 'wheel_neg' : {'slope' : 0.430013*100, 'intercept' : 0.215000*100}}
        self.veh_params = {'R': 0.031, 'L': 0.150, 'wheel_pos': {'slope': 0.350013 * 100, 'intercept': -
                                                                 0.205093 * 100}, 'wheel_neg': {'slope': 0.4050013 * 100, 'intercept': 0.205000 * 100}}
        # self.veh_params = {'R' : 0.031, 'L' : 0.150, 'wheel_pos' : {'slope' : 0.360013*100, 'intercept' : -0.215093*100}, 'wheel_neg' : {'slope' : 0.360013*100, 'intercept' : 0.215000*100}}
        self.left_to_right_ratio = 1

        self.move = 0.0
        self.stop = 1.0

        self.timestep = 0.1

        print("INITIALIZED")

    def stop_motion(self):
        msg = Float32MultiArray()
        msg.data = [self.stop, 0, 0]
        self.pub.publish(msg)

    def map_phi_to_m(self, phi):
        dir = 0
        if phi > 0.5:
            dir = 1
            return (phi - self.veh_params['wheel_pos']['intercept']) / self.veh_params['wheel_pos']['slope']
        if phi < -0.5:
            dir = -1
            return (phi - self.veh_params['wheel_neg']['intercept']) / self.veh_params['wheel_neg']['slope']
        else:
            return 0

    def move_vw(self, v, w, delT):
        phi_r = w * self.veh_params['L'] / \
            (2 * self.veh_params['R']) + v / self.veh_params['R']
        phi_l = -w * self.veh_params['L'] / \
            (2 * self.veh_params['R']) + v / self.veh_params['R']
        motor_left, motor_right = self.map_phi_to_m(
            phi_l), self.map_phi_to_m(phi_r)
        steps = int(delT / self.timestep)

        # print(motor_left, motor_right, phi_l, phi_r)

        # print(steps, delT, self.timestep, delT/self.timestep)

        t1 = time.time()
        total_time = 0.0
        while total_time <= delT:
            msg = Float32MultiArray()
            msg.data = [self.move, (-motor_left *
                        self.left_to_right_ratio)/1.01, -motor_right/1.01]
            self.pub.publish(msg)
            time.sleep(self.timestep)
            total_time += self.timestep

        # print(total_time, time.time() - t1)
        msg = Float32MultiArray()
        msg.data = [self.stop, -0.4, -0.4]
        self.pub.publish(msg)

        # time.sleep(0.05)

        # time.sleep(0.0005)


def getV(dist, timestep, direction):
    v = dist / timestep
    if v > 0.25:
        return 0.25 * direction, timestep
    # elif v < 0.15:
    #     return getV(dist, timestep/2)
    return v * direction, timestep


def getW(angle, timestep):
    omega = angle / timestep
    # print(omega)
    omega_dir = omega / (abs(omega + 10**-10))
    if abs(omega) > math.pi / 3:
        return math.pi / 3 * omega_dir, timestep

    return omega, timestep
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

def check_do(v, omega, t1, t2):
    # print("CHECK DO", v, omega, t1, t2)
    if abs(v) < 0.25 and abs(omega) < math.pi * 3 / 2:
        # print("IMPOSSIBLE TO MOVE")
        return check_do(v * 2, omega * 2, t1 / 2, t2 / 2)
    return v, omega, t1, t2


class Planner:

    def __init__(self, waypoints):
        self.ctrl_pub = rospy.Publisher(
            '/ctrl_cmd', Float32MultiArray, queue_size=2)
        self.sub = rospy.Subscriber("/current_pose", Pose, self.pose_callback)

        self.cur_pose = {'t': np.asarray([0.0, 0.0]), 'r': 0.0}

        self.cur_waypoint = 0
        self.waypoints = waypoints
        self.timestep = 0.10

        self.cur_wp_reached = False

        self.flag = 0

        self.which_motion = 't'

        self.done = False

        self.controller_obj = MotorController()

        self.flagx = 0

    ''' A few helper functions '''

    def pose_callback(self, msg):
        '''
        Sets the current pose
        '''

        print("IN CALLBACK")
        cur_pose_matrix = np.asarray(msg.pose.matrix).reshape((3, 4))

        trans = cur_pose_matrix[:3, 3]
        rot = cur_pose_matrix[:3, :3]
        rot_y = rotat_to_euler(rot)[1]

        self.cur_pose['t'][0] = trans[2]
        self.cur_pose['t'][1] = -trans[0]
        self.cur_pose['r'] = rot_y

        if self.cur_wp_reached:
            self.cur_wp_reached = False
            # time.sleep(2.0)
            return

        self.flagx = 1
        if self.done == False:
            self.runner()

    def runner(self):

        if self.which_motion == 't':
            cur_xy = self.cur_pose['t']
            cur_ori = self.cur_pose['r']

            goal_pose = self.waypoints[self.cur_waypoint]
            goal_xy = goal_pose[:2]
            goal_ori = goal_pose[2]

            dist = math.sqrt((goal_xy[0] - cur_xy[0])
                             ** 2 + (goal_xy[1] - cur_xy[1])**2)
            if self.which_motion == 'r':
                dist = 0

            new_vector = np.matmul(np.asarray([[math.cos(cur_ori), -math.sin(cur_ori)], [
                                   math.sin(cur_ori), math.cos(cur_ori)]]), np.asarray(cur_xy).reshape((2, 1)))

            direction = -1 if goal_xy[0] - new_vector[0] > 0 else 1
            # if self.flag == 1:
            #     cur_xy = self.waypoints[0][:2]

            theta = math.atan2(goal_xy[1] - cur_xy[1],
                               goal_xy[0] - cur_xy[0]) - cur_ori

            theta_dir = theta / (abs(theta + 10**-10))
            theta = abs(theta) % (math.pi * 2) * theta_dir

            if abs(theta) > math.pi:
                theta = math.pi * 2 - abs(theta + 10**-10)
                theta *= theta_dir * -1

            print("DIST", dist, theta, cur_ori,
                  theta + cur_ori, cur_xy, goal_xy)

            if ((self.flag == 0) or (abs(dist) > 0.05)):
                v, t1 = getV(dist * 0.10, self.timestep, direction)
                omega, t2 = getW(theta * max(dist, 0.50) * 0.3, self.timestep)

                if abs(dist) < 0.05:

                    self.flag = 1

                # if self.flag == 1:
                #     v = v/5
                #     t2/=8
                #     t1 = t2
                #     omega *= 10
                #     omega/=dist

                # print(v, omega, theta, dist)
                v, omega, t1, t2 = check_do(v, omega, t1, t2)
                # omega = 0.0
                print(v, omega, theta, dist, cur_ori, goal_ori, cur_xy)

                # self.controller_obj.move_vw(0, -omega*3, (t1+t2)/4)

                self.controller_obj.move_vw(v, -omega * 2, (t1 + t2) / 2)
                # time.sleep(0.2)
                # self.update_pose(v, omega , (t1+t2)/2)

            else:
                cur_ori = self.cur_pose['r']
                print("REACHED", dist, cur_ori)
                self.controller_obj.stop_motion()
                self.cur_waypoint += 1
                self.which_motion = 'r'
                # time.sleep(2.0)
                self.execute_in_place_rotation()
                # time.sleep(2.0)
                self.execute_in_place_rotation()

        elif self.which_motion == 'r':

            print("DOING ROTATIONAL MOTION ", self.cur_waypoint)
            print(self.waypoints)
            cur_ori = self.cur_pose['r']
            goal_pose = self.waypoints[self.cur_waypoint]

            goal_ori = goal_pose[2]
            theta = goal_ori - cur_ori

            theta_dir = theta / (abs(theta + 10**-10))
            theta = abs(theta) % (math.pi * 2) * theta_dir

            if abs(theta) > math.pi:
                theta = math.pi * 2 - abs(theta + 10**-10)
                theta *= theta_dir * -1
            print("DONE??", theta, goal_ori, cur_ori)

            if ((abs(theta) > 0.020)):
                omega, t2 = getW(theta * 0.2, self.timestep)
                v, omega, t1, t2 = check_do(0, omega, t2, t2)
                print("EXEC", v, omega, t2)
                self.controller_obj.move_vw(v, -omega, (t2 + t2) / 2)
                time.sleep(0.3)

            else:
                print("WAYPOINT 2 reached")
                self.controller_obj.stop_motion()
                self.cur_waypoint += 1
                self.which_motion = 't'

        if self.cur_waypoint == len(self.waypoints):
            self.done = True

    def execute_in_place_rotation(self):
        cur_ori = self.cur_pose['r']
        self.controller_obj.stop_motion()
        # self.cur_waypoint+=1

        # time.sleep(2.0)

        self.cur_wp_reached = True
        nextwp = self.waypoints[self.cur_waypoint]
        thetadiff = nextwp[2] - cur_ori

        #TODO - omegadirection
        omegadirection = 1

        omeganow = - np.pi * 3 / 2 * omegadirection
        timenow = abs(thetadiff) / abs(omeganow)

        additional_startup_time = 0.15
        if nextwp[2] == 0:
            additional_startup_time
            omeganow = 0.0

        print(thetadiff, omeganow, timenow)

        self.controller_obj.move_vw(
            0, -omeganow, timenow + additional_startup_time)
        # self.cur_waypoint+=1

        # time.sleep(2.0)
        print("DONE HERE")

        print(self.cur_waypoint)
        self.which_motion = 't'
        self.cur_waypoint += 1
        # time.sleep(2.0)

    def update_pose(self, v, omega, timstep):

        self.cur_pose['t'][0] += v * np.cos(omega) * timstep
        self.cur_pose['t'][1] += v * np.sin(omega) * timstep
        self.cur_pose['r'] = omega * timstep

        self.flagx = 2


if __name__ == "__main__":
    rospy.init_node('planner_node')

    # # ccsv = CSVReader("waypoints.txt")
    # # waypoints = ccsv.read_and_parse_file()
    # # waypoints = ccsv.process_waypoints()

    # waypoints = [[1.0,0,0], [1.0,0,-np.pi/2], [1.0, -0.61, -np.pi/2]]
    waypoints = [[1.0, 0, 0], [1.0, 0, 0], [1.0, 0, -np.pi / 2], [2.00, 0, 0], [2.00, 0, -np.pi / 2], [2.00, 0, -
                                                                                                       (np.pi / 2 - 0.4636)], [math.sqrt(5), 0, 0], [math.sqrt(5), 0, 0], [math.sqrt(5), 0, -(np.pi / 2 + 0.4636)]]
    # waypoints = [ [1.0, -0.61, -np.pi/2]]
    # waypoints = [ [0.63, 0, 0]]
    # waypoints = [[0.5,0,0], [0.5,0,0]]

    p = Planner(waypoints)
    time.sleep(1.0)
    # while True:
    #    p.runner()
    #    time.sleep(0.1)
    # rospy.spin()

    # m = MotorController()
    # for i in range(5):
    # m.move_vw(0, 3.14*2, 2.0)
    rospy.spin()

    # 3.2 -
    # 2.1 - 1.3cm and 1.8cm
    # 4.1 - 3.1 cm x, 2.3 cm y
