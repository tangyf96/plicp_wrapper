#! /usr/bin/python3
# -*- coding: UTF-8 -*-

import re
import sys
import getopt
import linecache
import matplotlib.pyplot as plt
import numpy as np

class displayPath:
    def __init__(self, filename, filename2, filename3):
        self.filename = filename 
        self.odom_filename = filename2
        self.result_file = filename3

        self.laser_x = []
        self.laser_y = []
        self.delta_laser_x = []
        self.delta_laser_y = []

        self.timestamp = []
        self.odom_x = []
        self.odom_y = []
        # laser_to_odom transformation
        dx = 0.051
        dy = 0
        dtheta = np.deg2rad(-118)
        self.T_laser_to_odom =  self.rotationMatrix(dx, dy, dtheta)

    def rotationMatrix(self, dx, dy, dtheta):
        T = np.array([[np.cos(dtheta), -np.sin(dtheta), dx], 
                                      [np.sin(dtheta), np.cos(dtheta), dy], 
                                      [0, 0, 1]])
        return T

    def calculate(self):
        # 根据激光的帧间数据，计算odom坐标系下的轨迹
        with open(self.filename, 'r') as file:
            lidar_cnt = 0
            for line in file.readlines():
                if lidar_cnt == 0:
                    self.laser_x.append(0)    
                    self.laser_y.append(0)
                    self.prevT = np.eye(3)
                else:
                    array = line.split(",", 5)
                    dx = float(array[0])
                    dy = float(array[1])
                    dtheta = float(array[2])
                    time = float(array[3])
                    print("laser frame data {t}, {x}, {y}, {theta}".format(t=time, x=dx, y=dy, theta=dtheta))
                    # laser frame transformation
                    deltaT_laser = self.rotationMatrix(dx, dy, dtheta)
                    # odom frame transformation
                    T_odom = np.dot(self.T_laser_to_odom, np.dot(deltaT_laser, np.linalg.inv(self.T_laser_to_odom)))
                    # world frame transformation
                    curT = self.prevT.dot(T_odom)
                    
                    self.laser_x.append(curT[0,2])
                    self.laser_y.append(curT[1,2])
                    self.delta_laser_x.append(T_odom[0][2])
                    self.delta_laser_y.append(T_odom[1][2])
                    print("world frame data ({x}, {y})".format(x=self.laser_x[-1], y=self.laser_y[-1]))
                    self.timestamp.append(time)
                    self.prevT = curT
                lidar_cnt = lidar_cnt + 1
        
        # 根据odom数据计算轨迹
        with open(self.odom_filename, 'r') as file:
            odom_cnt = 0
            for line in file.readlines():
                if odom_cnt == 0:
                    self.odom_x.append(0)    
                    self.odom_y.append(0)
                    self.prevT = np.eye(3)
                else:
                    array = line.split(" ", 4)
                    dx = float(array[0])
                    dy = float(array[1])
                    dtheta = float(array[2])
                    deltaT = self.rotationMatrix(dx, dy, dtheta)
                    curT = self.prevT.dot(deltaT)
                    self.odom_x.append(curT[0,2])
                    self.odom_y.append(curT[1,2])
                    self.prevT = curT
                odom_cnt = odom_cnt + 1
                if odom_cnt > lidar_cnt:
                    break

    def display(self):
        # 显示轨迹结果
        plt.plot(self.laser_x, self.laser_y, 'b.-')
        plt.plot(self.odom_x, self.odom_y, 'r.-')
        plt.plot(0, 0, 'ro')
        plt.title("path")
        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        plt.xlim((-0.5,1.5))
        plt.ylim((-1,2))
        plt.show()

    def save_laser_delta_data(self):
        # 保存laser_scan的里程计帧间的位移数据
        output_file = open(self.result_file, 'w')
        for i in range(len(self.delta_laser_x)):
            output_str = str(self.delta_laser_x[i]) + " " + str(self.delta_laser_y[i]) + " " + str(self.timestamp[i]) + "\n"
            output_file.write(output_str)

def main():
    filename = "/home/yifan/calibration/plicp/result/data.txt"
    filename2 = "/home/yifan/calibration/plicp/data/f_trfm_odom.txt"
    filename3 = "/home/yifan/calibration/plicp/result/delta_laser.txt"

    handle = displayPath(filename, filename2, filename3)
    handle.calculate()
    handle.save_laser_delta_data()
    handle.display()

if __name__ == "__main__":
    main()
