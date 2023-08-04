# coding=gbk
import re
import numpy as np


class Data:
    def __init__(self):
        self.customerNum = 0
        self.nodeNum = 0
        self.vehicleNum = 0
        self.capacity = 0
        self.vehicle_speed = 0
        self.uav_speed = 0
        self.uav_range = 0
        self.uav_accessible = []
        self.cor_X = []
        self.cor_Y = []
        self.demand = []
        self.serviceTime = []
        self.readyTime = []
        self.dueTime = []
        self.disMatrix = []
        self.arcs = {}

def clip_data(data, n):
    data.customerNum = n
    data.nodeNum = n + 2
    data.uav_accessible = data.uav_accessible[:n+1]
    data.cor_X = data.cor_X[:n+1]
    data.cor_Y = data.cor_Y[:n+1]
    data.demand = data.demand[:n+1]
    data.serviceTime = data.serviceTime[:n+1]
    data.readyTime = data.readyTime[:n+1]
    data.dueTime = data.dueTime[:n+1]
    data.disMatrix = np.array(data.disMatrix)[:n+1, :n+1].tolist()
    return data

def readData(data, path):
    f = open(path, 'r')
    lines = f.readlines()
    count = 0
    for line in lines:
        count = count + 1
        if count == 1:  # 客户数量
            line = line[:-1].strip()
            data.customerNum = int(line)
            data.nodeNum = data.customerNum + 2
        if count == 3:  # 车辆容量
            line = line[:-1].strip()
            data.capacity = int(line) / 2
        if count == 4:  # 车辆速度
            line = line[:-1].strip()
            data.vehicle_speed = 8.5  # m/s
        if count == 5:  # uav速度
            line = line[:-1].strip()
            data.uav_speed = 15  # 巡航速度15m/s
        if count == 6:  # uav 里程
            line = line[:-1].strip()
            data.uav_range = 10  # 15  # KM  最大续航可为20

        if 8 <= count <= 8 + data.customerNum:
            line = line[:-1].strip()
            str = re.split('\t', line)
            data.disMatrix.append(list(map(int, str)))  # 单位 m

        if count == 11 + data.customerNum:  # depot info
            line = line[:-1].strip()
            str = re.split('\t', line)
            if len(str) != 8:
                print(str)
            assert len(str) == 8
            data.uav_accessible.append(int(0))
            data.demand.append(float(0))
            data.readyTime.append(float(0))   # float(str[4])
            data.dueTime.append(float(0))
            data.serviceTime.append(float(0) * 10)  # 服务时间扩大了10倍
        if count >= 13 + data.customerNum:
            line = line[:-1].strip()
            # str = re.split('\t', line)
            str = re.split("\\s+", line)
            if len(str) != 8:
                print(str)
            assert len(str) == 8
            data.uav_accessible.append(int(str[2]))
            data.demand.append(float(str[3]))
            data.readyTime.append(float(str[4]))
            data.dueTime.append(float(str[5]))
            data.serviceTime.append(float(str[6]) * 10)  # 服务时间扩大了10倍


if __name__ == '__main__':
    data = Data()
    path = f"data/data15/Cardiff15_01.txt"
    readData(data, path)
    data2 = clip_data(data, 10)
    test = 1
    a = 10
