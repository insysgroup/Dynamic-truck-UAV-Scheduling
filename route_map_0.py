import random
from env import configurations as con
import matplotlib.pyplot as plt
import networkx as nx

from read_Data import Data, readData


class RouteMap:
    def __init__(self, item):
        rate = 0.4
        data_path = f"env/data/data100/Cardiff100_0{item}.txt"
        nodenum= None
        self.data = Data()
        readData(self.data, path=data_path)
        if nodenum is None:
            self.node_num = self.data.customerNum + 1  # 包含depot
        else:
            self.node_num = nodenum
        self.truck_num = con.truck_num
        self.uav_num = con.uav_num
        self.max_uav_num = con.max_uav_num
        self.total_uav_num = self.uav_num * self.truck_num
        self.total_truck_capacity = self.data.capacity
        self.X = self.data.cor_X[:self.node_num]  # 地图长
        self.Y = self.data.cor_Y[:self.node_num]  # 地图宽
        self.v_k = self.data.vehicle_speed  # 货车速度 m/s
        self.v_u = self.data.uav_speed  # 无人机速度
        self.fli_range = self.data.uav_range * 1000 / self.v_u
        self.demand = self.data.demand[:self.node_num]  # 需求所需的物资量
        self.service_time = self.data.serviceTime[:self.node_num]  # 服务时长
        self.require_time = self.data.readyTime[:self.node_num]  # 将算例的readyTime设置为惩罚开始时间
        self.disMatrix = [self.data.disMatrix[i][:self.node_num] for i in range(self.node_num)]
        self.truck_adj_matrix = [[0 for j in range(self.node_num)] for i in range(self.node_num)]
        self.uav_adj_matrix = [[0 for j in range(self.node_num)] for i in range(self.node_num)]
        self.graph = self.build_graph(destroy_rate=rate)

    def build_graph(self, destroy_rate):
        Graph = nx.DiGraph()
        node_col = {}
        for i in range(self.node_num):
            node_col[str(i)] = 'gray'
            note_type = 'customer'
            if (i == 0):
                note_type = 'depot'
                node_col[str(i)] = 'red'
            Graph.add_node(
                str(i),
                ID=i,
                note_type=note_type,
                arrive_time=100000,
                demand=self.demand[i],
                serve_time=self.service_time[i],
                required_time=self.require_time[i],
                min_dis=0,
                previous_node=None
            )
        # 添加最后的虚拟节点
        node_col[str(self.node_num)] = 'red'
        Graph.add_node(
            str(self.node_num),
            ID=self.node_num,
            note_type='depot',
            arrive_time=100000,
            demand=self.demand[0],
            serve_time=self.service_time[0],
            required_time=self.require_time[0],
            min_dis=0,
            previous_node=None
        )
        # 添加边
        edge_list = []


        for i in range(0, self.node_num):
            for j in range(i, self.node_num):
                edge_list.append([i, j])
        destroy_num = int(destroy_rate * len(edge_list))
        random.seed(1)
        destroy_list = random.sample(edge_list, destroy_num)
        for [i, j] in destroy_list:
            self.disMatrix[i][j] = 100000
            self.disMatrix[j][i] = 100000
        for i in range(self.node_num + 1):
            for j in range(self.node_num + 1):
                if (1 <= i <= self.node_num - 1) and (1 <= j <= self.node_num - 1):
                    if (i != j):
                        Graph.add_edge(str(i), str(j),
                                       truck_time = self.get_truck_travel_time(i, j),
                                       uav_time = self.get_uav_travel_time(i, j))
                if i == 0 and (1 <= j <= self.node_num - 1):
                    Graph.add_edge(str(i), str(j),
                                   truck_time=self.get_truck_travel_time(i, j),
                                   uav_time=self.get_uav_travel_time(i, j))
                if (1 <= i <= self.node_num - 1) and j == self.node_num:
                    Graph.add_edge(str(i), str(j),
                                   truck_time=self.get_truck_travel_time(i, 0),
                                   uav_time=self.get_uav_travel_time(i, 0))
            for [i, j] in destroy_list:
                if [i, j] in Graph:
                    Graph.remove_edge(str(i), str(j))
                if [j, i] in Graph:
                    Graph.remove_edge(str(j), str(i))

        return Graph

    def plot_scatter(self, location):
        for i in range(1, len(location)):
            plt.scatter(location[i][0], location[i][1], c='b')  # iot
        plt.plot(location[0][0], location[0][1], c='r', marker='s')  # central station
        plt.show()

    def get_truck_travel_time(self, start: int, end: int):
        travel_time = self.disMatrix[start][end] / self.v_k
        return travel_time

    def get_uav_travel_time(self, start: int, end: int):
        travel_time = self.disMatrix[start][end] / self.v_u
        return travel_time
