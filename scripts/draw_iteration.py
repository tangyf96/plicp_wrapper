#!/usr/bin/python
# -*- coding: UTF-8 -*-
import matplotlib.pyplot as plt
from matplotlib.patches import ConnectionPatch
import math
import numpy as np

class DrawIteration():
    def __init__(self, result_name, pre):
        self.filename = result_name        
        self.pre = pre

    def read_data_ref(self, data_file):
        ref_x = []
        ref_y = []
        with open(data_file, 'r') as infile:
                for line in infile:
                    pose = [ float(i) for i in line.split()]
                    ref_x.append(pose[0])
                    ref_y.append(pose[1])
        return ref_x, ref_y

    def run(self):
        target_time = 9181.671471548
        # open result file
        with open(self.filename, "r") as infile:
            find_new_time = False
            num_laser_points = 0
            itr_num = 0
            cur_num = 0
            for line in infile:
                # find the start of data at a timestamp 
                if not find_new_time:
                    if 'num' in line:
                        find_new_time = True
                        num_laser_points = int(line.split(" ", 2)[1])
                        cur_num = 0
                        ref_x = []
                        ref_y = []
                        points_x = []
                        points_y = []
                        ind1 = []
                        ind2 = []
                    else:
                        continue
                else:
                    # Update the timestamp and iteration number
                    if cur_num is 0:
                        time_stamp = line.split("-", 4)[1]
                        itr_num = int(line.split("-", 4)[-1])

                        ref_file_name  = self.pre + "ref-" + time_stamp + ".txt"
                        ref_x, ref_y = self.read_data_ref(ref_file_name)
                        sen_file_name = self.pre + "sen-" + time_stamp + ".txt"
                        sen_x, sen_y = self.read_data_ref(sen_file_name)

                        # 第一次的时候显示reference 和 measurement
                        if itr_num is 0 and math.isclose(float(time_stamp), target_time):
                            # open reference data
                            fig = plt.figure(1)
                            plt.plot(ref_x, ref_y, 'r.')
                            plt.plot(sen_x, sen_y, 'g.')
                            title = "ref-sen-" + time_stamp
                            plt.title(title)
                    else:
                        # read the next num lines as laser_sens
                        array  = line.split(" ", 5)
                        x_world = float(array[0])
                        y_world = float(array[1])
                        match_ind1 = int(array[2])
                        match_ind2 = int(array[3])

                        # 插入匹配
                        points_x.append(x_world)
                        points_y.append(y_world)
                        ind1.append(match_ind1)
                        ind2.append(match_ind2)

                        # 当前帧已经匹配完成
                        if cur_num == num_laser_points:
                            find_new_time = False
                            # 显示当前帧的correspondence
                            # 只有在相应时间的时候才展示
                            if math.isclose(float(time_stamp), target_time):
                                fig = plt.figure(2)
                                plt.plot(ref_x, ref_y, 'r.')
                                plt.plot(points_x, points_y, 'g.')
                                title = "ref-sen-" + time_stamp + " iter: " + str(itr_num)
                                plt.title(title)
                                
                                self.display_correspondence(ref_x, ref_y, points_x, points_y, ind1, ind2, time_stamp, itr_num)

                        # end if
                    # end if
                    cur_num += 1

                # end if
            # end for
        # end with

    def display_correspondence(self, ref_x, ref_y, points_x, points_y, ind1, ind2, time_stamp, itr_num):
        fig = plt.figure(figsize=(10,10))
        ax2 = fig.add_subplot(122)
        ax1 = fig.add_subplot(121)

        ax1.plot(ref_x, ref_y, '.r')
        cnt = 0
        corr_cnt = 0
        for i in range(len(points_x)):
            ax2.plot(points_x[i], points_y[i], '.g')
            if ind1[i] == -1 or ind2[i] == -1:
                continue
            else:
                corr_cnt += 1
            # if i % 10 == 0: # 只选取一部分显示匹配，防止过多看不清
                ref_point = (ref_x[ind1[i]], ref_y[ind1[i]])
                sen_p = (points_x[i], points_y[i])
                con = ConnectionPatch(xyA=ref_point, xyB=sen_p, coordsA="data", coordsB="data",
                                                                axesA=ax1, axesB=ax2, linestyle='--', color='blue')
                ax1.add_artist(con)

                ref_point = (ref_x[ind2[i]], ref_y[ind2[i]])
                con = ConnectionPatch(xyA=ref_point, xyB=sen_p, coordsA="data", coordsB="data",
                                                axesA=ax1, axesB=ax2, linestyle='--', color='k')
                ax1.add_artist(con)
                cnt += 1
        print("print correspondence: ", cnt)
        title = "correspondence time: " + str(time_stamp) + " iteration: " + str(itr_num) + " corr: " + str(corr_cnt)
        plt.title(title)
        plt.xlim((-3.0, 1.5))
        plt.ylim((-2.0, 2.0))
        plt.show()

if __name__ =='__main__':
    niter = 13
    filename = '/home/yifan/calibration/plicp/build/result.txt'
    pre = '/home/yifan/calibration/plicp/result/'

    handle = DrawIteration(filename, pre)
    handle.run()