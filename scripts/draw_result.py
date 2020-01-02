#! /usr/bin/python3
# -*- coding: UTF-8 -*-

import re
import sys
import getopt
import linecache
import matplotlib.pyplot as plt
import numpy as np

class displayPath:
    def __init__(self, filename, filename2):
        self.filename = filename
        self.odom_filename = filename2
        self.x = []
        self.y = []
        self.odom_x = []
        self.odom_y = []
        self.offsetT = self.rotationMatrix(0, 0, np.deg2rad(-118))

    def rotationMatrix(self, dx, dy, dtheta):
        T = np.array([[np.cos(dtheta), -np.sin(dtheta), dx], 
                                      [np.sin(dtheta), np.cos(dtheta), dy], 
                                      [0, 0, 1]])
        return T

    def calculate(self):
        with open(self.filename, 'r') as file:
            num = 1
            for line in file.readlines():
                if num == 1:
                    self.x.append(0)    
                    self.y.append(0)
                    self.prevT = np.eye(3)
                else:
                    array = line.split(",", 5)
                    dx = float(array[0])
                    dy = float(array[1])
                    dtheta = float(array[2])

                    deltaT = self.rotationMatrix(dx, dy, dtheta)
                    
                    curT = self.prevT.dot(deltaT)

                    rotT = np.dot(self.offsetT, curT)
                    self.x.append(curT[0,2])
                    self.y.append(curT[1,2])
                    self.prevT = curT
                num = num + 1
        
        with open(self.odom_filename, 'r') as file:
            num = 1
            for line in file.readlines():
                if num == 1:
                    self.odom_x.append(0)    
                    self.odom_y.append(0)
                    self.prevT = np.eye(3)
                else:
                    array = line.split(",", 4)
                    dx = float(array[0])
                    dy = float(array[1])
                    dtheta = float(array[2])
                    deltaT = self.rotationMatrix(dx, dy, dtheta)
                    curT = self.prevT.dot(deltaT)
                    self.odom_x.append(curT[0,2])
                    self.odom_y.append(curT[1,2])
                    self.prevT = curT
                num = num + 1

    def display(self):
        plt.plot(self.x, self.y, 'bo-')
        plt.plot(self.odom_x, self.odom_y, 'r-')
        plt.plot(0, 0, 'ro')
        plt.title("path")
        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        plt.show()

def main():
    filename = "/home/yifan/calibration/plicp/result/data.txt"
    filename2 = "/home/yifan/calibration/plicp/result/odom.txt"
    handle = displayPath(filename, filename2)
    handle.calculate()
    handle.display()

if __name__ == "__main__":
    main()

