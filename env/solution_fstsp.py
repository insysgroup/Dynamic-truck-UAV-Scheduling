#!/usr/bin/python
# -*- coding: utf-8 -*-
import random
import math
import copy
import networkx as nx
import dgl
import torch
import numpy as np
import matplotlib.pyplot as plt

from env.route_map import RouteMap
from env import configurations as con


class Solution:

    def __init__(self, route_map: RouteMap):
        # 方案类
        self.truck_time = None
        self.route_map = route_map
        self.truck_num = route_map.truck_num
        self.uav_num = route_map.uav_num
        self.total_serve_time = 0
        self.total_over_time = 0
        self.total_cost = 0
        self.solution_graph = None
        # 解方案
        # 卡车路径相关
        self.truck_route = {}  # k: []
        self.node_truck = {}  # node_id: truck_id
        self.truck_capacity = {}
        self.truck_uav_num = {}
        self.uav_node = {}  # d: s, e
        self.uav_take_off = {}  # s: (d, e)
        self.uav_land = {}  # e: (s, d)

    def init_routes(self, seed):
        # 考虑货物容量, 随机给node分配货车
        random.seed = seed
        temp_capacity = [self.route_map.total_truck_capacity for i in range(self.truck_num)]
        for i in range(1, self.route_map.node_num):
            while True:
                k = random.randint(0, self.truck_num - 1)
                if self.route_map.demand[i] <= temp_capacity[k]:
                    break
            self.node_truck[i] = k
            temp_capacity[k] = temp_capacity[k] - self.route_map.demand[i]
        # 相同货车的node集合 --> 成为一个货车路径
        truck_num_list = [[0] for i in range(self.truck_num)]
        for i in range(1, self.route_map.node_num):
            truck_num_list[self.node_truck[i]].append(i)
        for i in range(len(truck_num_list)):
            self.truck_route[i] = truck_num_list[i]
        # 寻找可行的无人机路径
        # init truck_uav_num_list
        truck_uav_num_list = [[] for i in range(self.truck_num)]
        for k in range(self.truck_num):
            for i in range(len(self.truck_route[k])):
                truck_uav_num_list[k].append(self.uav_num)
        for k in range(self.truck_num):
            for i in range(len(self.truck_route[k])):
                capacity, uav_num = self._check_truck_route(self.truck_route[k])
                if uav_num is not None:
                    if (i >= 2) & (i < len(self.truck_route[k]) - 1) & (len(self.truck_route[k]) > 3):  # 无人机不可以从depot出发
                        s = self.truck_route[k][i - 1]
                        d = self.truck_route[k][i]
                        j = self.truck_route[k][i + 1]
                        if self.judge_fli_range(s, d, j):  # 满足里程约束
                            del self.node_truck[d]
                            del self.truck_route[k][i]
                            self.uav_node[d] = (s, j)
                            self.uav_take_off[s] = (d, j)
                            self.uav_land[j] = (s, d)
                            _, self.truck_uav_num[k] = self._check_truck_route(self.truck_route[k])
        for i in range(self.truck_num):
            self.truck_uav_num[i] = truck_uav_num_list[i]

        # 计算货车离开每个节点时的货物剩余量
        truck_capacity_list = [[self.route_map.total_truck_capacity] for i in range(self.truck_num)]
        for k in range(self.truck_num):
            for i in range(len(self.truck_route[k])):
                if i != 0:
                    if self.truck_route[k][i] not in self.uav_take_off.keys():
                        truck_capacity_list[k].append(truck_capacity_list[k][i - 1] - self.route_map.demand[i])
                    else:
                        index_j = self.uav_take_off[self.truck_route[k][i]][0]
                        demand_j_capacity = self.route_map.demand[index_j]
                        truck_capacity_list[k].append(
                            truck_capacity_list[k][i - 1] - self.route_map.demand[i] - demand_j_capacity)
        for i in range(len(truck_num_list)):
            self.truck_capacity[i] = truck_capacity_list[i]
        # 更新时间参数
        for t, r in self.truck_route.items():
            self.truck_capacity[t], self.truck_uav_num[t] = self._check_truck_route(r)
            assert self.truck_capacity[t] is not None
            assert self.truck_uav_num[t] is not None
        self.solution_graph = self.creat_dgl_homograph()
        self.uav_node, self.uav_take_off, self.uav_land = self._check_uav_circle(self.solution_graph, self.uav_node,
                                                                                 self.uav_take_off,
                                                                                 self.uav_land)
        assert self.uav_node is not False
        self.cal_cost()

    def judge_fli_range(self, s, d, j):
        d = self.route_map.get_uav_travel_time(s, d) + self.route_map.get_uav_travel_time(d, j)
        if d > self.route_map.fli_range:
            return False
        else:
            return True

    def init_routes_2(self,seed):
        """1. 更具node的require_time排序，然后依次轮流分配给truck
           2. 针对卡车路径计算所有node的300超时服务时间，并排序
           3. 按照超时服务时间从大到小，依次指定可行的无人机辅组线路"""
        random.seed(seed)
        node_list = list(range(1, self.route_map.node_num))
        random.shuffle(node_list)
        r_length = self.route_map.node_num // self.truck_num
        truck_route = {}
        for truck in range(self.truck_num - 1):
            truck_route[truck] = node_list[truck * r_length:(truck + 1) * r_length]
            truck_route[truck] = sorted(truck_route[truck], key=lambda x: self.route_map.require_time[x])
            # truck_route[truck].insert(0, 0)
        truck_route[self.truck_num - 1] = node_list[(self.truck_num - 1) * r_length:]
        truck_route[self.truck_num - 1] = sorted(truck_route[self.truck_num - 1],
                                                 key=lambda x: self.route_map.require_time[x])
        # 加无人机： 从前往后，间隔添加
        for k, r in truck_route.items():
            self.truck_route[k] = [0]
            uav_node_index = -2
            for i in range(len(r)):
                if (i != 0 and i != len(r) - 1) and \
                        (i - 1 > uav_node_index + 1 and self.judge_fli_range(r[i - 1], r[i], r[i + 1])):
                    self.uav_node[r[i]] = (r[i - 1], r[i + 1])
                    self.uav_take_off[r[i - 1]] = (r[i], r[i + 1])
                    self.uav_land[r[i + 1]] = (r[i - 1], r[i])
                    uav_node_index = i
                else:
                    self.truck_route[k].append(r[i])
                    self.node_truck[r[i]] = k
        for t, r in self.truck_route.items():
            self.truck_capacity[t], self.truck_uav_num[t] = self._check_truck_route(r)
            assert self.truck_capacity[t] is not None
            assert self.truck_uav_num[t] is not None
        self.solution_graph = self.creat_dgl_homograph()
        self.uav_node, self.uav_take_off, self.uav_land = self._check_uav_circle(self.solution_graph, self.uav_node,
                                                                                 self.uav_take_off,
                                                                                 self.uav_land)
        assert self.uav_node is not False
        self.cal_cost()

    def creat_dgl_homograph(self):
        s_graph = dgl.DGLGraph()
        ndata = torch.tensor(np.full(self.route_map.node_num, np.nan))
        s_graph = dgl.add_nodes(s_graph, self.route_map.node_num)
        s_graph.ndata['a_time'], s_graph.ndata['l_time'], s_graph.ndata['s_time'], s_graph.ndata['dead_time'] = \
            copy.deepcopy(ndata), copy.deepcopy(ndata), copy.deepcopy(ndata), copy.deepcopy(ndata)
        s_graph.ndata['l_time'][0] = 0.
        s_graph.ndata['s_time'][0] = 0.
        s_graph.ndata['dead_time'][0] = 0.
        e_times = []
        node_num = 0
        for v, r in self.truck_route.items():
            if len(r) > 1:
                for i in range(1, len(r)):
                    s_graph.ndata['s_time'][r[i]] = self.route_map.service_time[r[i]]
                    s_graph.ndata['dead_time'][r[i]] = self.route_map.require_time[r[i]]
                    node_num += 1
                    t_time = self.route_map.get_truck_travel_time(r[i - 1], r[i])
                    e_times.append(t_time)
                    s_graph.add_edges(r[i - 1], r[i])
                t_time = self.route_map.get_truck_travel_time(r[-1], 0)
                e_times.append(t_time)
                s_graph.add_edges(r[-1], 0)

        for s, (d, e) in self.uav_take_off.items():
            u_time1 = self.route_map.get_uav_travel_time(s, d)
            u_time2 = self.route_map.get_uav_travel_time(d, e)
            s_graph.ndata['s_time'][d] = self.route_map.service_time[d]
            s_graph.ndata['dead_time'][d] = self.route_map.require_time[d]
            e_times.append(u_time1)
            e_times.append(u_time2)
            s_graph.add_edges(s, d)
            s_graph.add_edges(d, e)
            node_num += 1
        s_graph.edata['time'] = torch.tensor(e_times)
        return s_graph

    def cal_leave_time(self, node, s_graph):
        """
        :param node: 需计算leave_time的点
        :param s_graph: 行驶路径图，networkx对象, dgl对象
        :return:
        """
        # 开始计算
        predecessors = s_graph.predecessors(node).tolist()
        if node in self.uav_node.keys():  # 点为无人机访问的点
            assert len(predecessors) == 1
            pre_node = predecessors[0]
            last_a_time = s_graph.ndata['a_time'][pre_node]
            if np.isnan(last_a_time):  # 递归
                self.cal_leave_time(pre_node, s_graph)
            assert not np.isnan(last_a_time)
            s_graph.ndata['a_time'][node] = s_graph.ndata['a_time'][pre_node] \
                                            + self.route_map.get_uav_travel_time(pre_node, node)
            s_graph.ndata['l_time'][node] = s_graph.ndata['a_time'][node] + s_graph.ndata['s_time'][node]
        else:  # 点为卡车点
            pre_l_times = []
            for pre_node in predecessors:
                last_l_time = s_graph.ndata['l_time'][pre_node]
                if np.isnan(last_l_time):  # 递归
                    self.cal_leave_time(pre_node, s_graph)
                assert not np.isnan(last_l_time)
                if pre_node in self.uav_node.keys():
                    pl_time = last_l_time + self.route_map.get_uav_travel_time(pre_node, node)
                    pre_l_times.append(pl_time)
                elif node != 0:  # 有且仅有一个
                    assert np.isnan(s_graph.ndata['a_time'][node])
                    s_graph.ndata['a_time'][node] = last_l_time + self.route_map.get_truck_travel_time(pre_node, node)
                    pre_l_times.append(s_graph.ndata['a_time'][node] + s_graph.ndata['s_time'][node])
                else:
                    pre_l_times.append(last_l_time + self.route_map.get_truck_travel_time(pre_node, node))
            if node == 0:
                s_graph.ndata['a_time'][node] = max(pre_l_times)
            else:
                s_graph.ndata['l_time'][node] = max(pre_l_times)
            self.truck_time = sum(pre_l_times)
        self.solution_graph = s_graph

    def cal_graph_time(self):
        # 重置图中部分属性
        ndata = torch.tensor(np.full(self.route_map.node_num, np.nan))
        self.solution_graph.ndata['a_time'], self.solution_graph.ndata['l_time'] = \
            copy.deepcopy(ndata), copy.deepcopy(ndata)
        self.solution_graph.ndata['l_time'][0] = 0.
        # s_graph.ndata['s_time'][0] = 0.
        self.cal_leave_time(0, self.solution_graph)
        total_serve_time = self.solution_graph.ndata['a_time'][0]
        self.solution_graph.apply_nodes(
            lambda nodes: {'over_time': torch.max(torch.tensor(0.), nodes.data['a_time'] - nodes.data['dead_time'])})
        total_over_time = dgl.readout_nodes(self.solution_graph, 'over_time', op='sum') - \
                          self.solution_graph.ndata['over_time'][0]
        total_cost = total_serve_time + con.alfa * total_over_time
        return total_cost.item(), total_over_time.item(), total_serve_time.item()

    def cal_cost(self):
        self.total_cost, self.total_over_time, self.total_serve_time \
            = self.cal_graph_time()
        return self.total_cost

    def truck_node_exchange_(self, node1, node2):  # 1  move 形式:(node1, node2) =  (node1, node2)
        """两个货车点交换，无人机路径不变"""
        if node1 in self.uav_take_off.keys() and node1 in self.uav_land.keys():
            print(r"存在一个点既起飞又降落的情况！！！")
            return None
        if node2 in self.uav_take_off.keys() and node2 in self.uav_land.keys():
            print(r"存在一个点既起飞又降落的情况！！！")
            return None
        node_truck = self.node_truck
        truck1 = node_truck[node1]
        truck2 = node_truck[node2]
        route1 = self.truck_route[truck1]
        if truck1 == truck2:
            route2 = route1
        else:
            route2 = self.truck_route[truck2]
        r_index1 = route1.index(node1)
        r_index2 = route2.index(node2)
        # 变更解
        route1[r_index1] = node2
        route2[r_index2] = node1
        node_truck[node1] = truck2
        node_truck[node2] = truck1

        # 判断交换后路径的可行性
        capacity, uav_num = self._check_truck_route(route1)
        capacity2, uav_num2 = capacity, uav_num
        if capacity is None:
            # return False
            return None
        else:
            self.truck_capacity[truck1] = capacity
            self.truck_uav_num[truck1] = uav_num
        if truck1 != truck2:  # 同一辆车交换
            return None
            capacity2, uav_num2 = self._check_truck_route(route2)
            if capacity2 is None:
                # return False
                return None
            else:
                self.truck_capacity[truck2] = capacity2
                self.truck_uav_num[truck2] = uav_num2

        # 检查无人机顺序
        uav_node = self.uav_node
        uav_take_off = self.uav_take_off
        uav_land = self.uav_land
        # 变更图

        if truck1 == truck2 and abs(r_index1 - r_index2) == 1:  # 两个点相邻
            n1, n2 = node1, node2
            r_1, r_2 = r_index1, r_index2
            if r_index1 > r_index2:  # node1在前
                n1, n2 = node2, node1
                r_1, r_2 = r_index2, r_index1
            delet_edges = [[route1[r_1 - 1], n1], [n1, n2]]
            add_edges = [[route1[r_1 - 1], n2], [n2, n1]]
            if len(route1) > r_2 + 1:
                delet_edges[0].append(n2)
                delet_edges[1].append(route1[r_2 + 1])
                add_edges[0].append(n1)
                add_edges[1].append(route1[r_2 + 1])
            else:
                delet_edges[0].append(n2)
                delet_edges[1].append(0)
                add_edges[0].append(n1)
                add_edges[1].append(0)
        else:  # 不相邻
            delet_edges = [[route1[r_index1 - 1], route2[r_index2 - 1]], [node1, node2]]
            add_edges = [[route1[r_index1 - 1], route2[r_index2 - 1]], [node2, node1]]
            if len(route1) > r_index1 + 1:
                delet_edges[0].append(node1)
                delet_edges[1].append(route1[r_index1 + 1])
                add_edges[0].append(node2)
                add_edges[1].append(route1[r_index1 + 1])
            else:
                delet_edges[0].append(node1)
                delet_edges[1].append(0)
                add_edges[0].append(node2)
                add_edges[1].append(0)
            if len(route2) > r_index2 + 1:
                delet_edges[0].append(node2)
                delet_edges[1].append(route2[r_index2 + 1])
                add_edges[0].append(node1)
                add_edges[1].append(route2[r_index2 + 1])
            else:
                delet_edges[0].append(node2)
                delet_edges[1].append(0)
                add_edges[0].append(node1)
                add_edges[1].append(0)
        delet_edge_id = self.solution_graph.edge_ids(delet_edges[0], delet_edges[1])
        self.solution_graph = dgl.remove_edges(self.solution_graph, delet_edge_id)
        self.solution_graph = dgl.add_edges(self.solution_graph, add_edges[0], add_edges[1])
        # 判断无人机方向可行性
        self.uav_node, self.uav_take_off, self.uav_land = self._check_uav_circle(self.solution_graph, uav_node,
                                                                                 uav_take_off, uav_land)
        if self.uav_node is False:
            return None
        return self.cal_cost()

    def truck_node_to_uav_(self, node):  # 2 move 形式: (node1=0, node2=node) 顺序可交换
        """将指定的truck访问的node变更为无人机访问"""
        truck = self.node_truck[node]
        r_index = self.truck_route[truck].index(node)
        if node in self.uav_take_off.keys() or node in self.uav_land.keys():
            return None
        if r_index == 0 or r_index == (len(self.truck_route[truck]) - 1):  # node 不为 depot 和 路径上最后一个节点
            return None
        if r_index == 1:  # 无人机不能从 depot 起飞
            return None
        s_node = self.truck_route[truck][r_index - 1]
        if s_node in self.uav_take_off.keys() or s_node in self.uav_land.keys():  # 前一个点已有无人机起飞
            return None
        if self.truck_uav_num[truck][r_index - 1] < 1:  # 无可用无人机
            return None
        e_node = self.truck_route[truck][r_index + 1]
        if e_node in self.uav_land.keys() or e_node in self.uav_take_off.keys():  # 后一个点已有无人机降落
            return None
        if self.truck_uav_num[truck][r_index + 1] >= self.route_map.uav_num:  # 后一点无法回收无人机
            return None
        if not self.judge_fli_range(s_node, node, e_node):
            return None

        # 更新交换后的解
        self.truck_route[truck].pop(r_index)
        del self.node_truck[node]
        self.truck_capacity[truck], self.truck_uav_num[truck] = self._check_truck_route(self.truck_route[truck])
        assert self.truck_capacity[truck] is not None
        assert self.truck_uav_num[truck] is not None
        # 无人机路径相关
        self.uav_node[node] = (s_node, e_node)
        self.uav_take_off[s_node] = (node, e_node)  # s: (d, e)
        self.uav_land[e_node] = (s_node, node)  # e: (s, d)
        # 变更图！！！！！！
        delet_edges = [[s_node, node], [node, e_node]]
        add_edges = [[s_node, s_node, node], [e_node, node, e_node]]
        delet_edge_id = self.solution_graph.edge_ids(delet_edges[0], delet_edges[1])
        self.solution_graph = dgl.remove_edges(self.solution_graph, delet_edge_id)
        self.solution_graph = dgl.add_edges(self.solution_graph, add_edges[0], add_edges[1])
        self.uav_node, self.uav_take_off, self.uav_land = \
            self._check_uav_circle(self.solution_graph, self.uav_node, self.uav_take_off, self.uav_land)
        if self.uav_node is False:
            return None
        return self.cal_cost()

    def uav_node_to_truck_(self, uav_route):  # 3  move 形式: (node1=s. node2=e)  可交换
        """无人机访问的点变成货车访问，判定约束后直接加到起飞点后"""
        s, d, e = uav_route
        truck1 = self.node_truck[s]
        truck2 = self.node_truck[e]
        s_index = self.truck_route[truck1].index(s)
        route1 = self.truck_route[truck1]
        if truck1 == truck2:
            route2 = route1
        else:
            route2 = self.truck_route[truck2]
        node_truck = self.node_truck
        uav_node = self.uav_node
        uav_take_off = self.uav_take_off
        uav_land = self.uav_land
        route1.insert(s_index + 1, d)
        node_truck[d] = truck1
        del uav_node[d]
        del uav_take_off[s]
        del uav_land[e]
        # 判断可行性
        capacity, uav_num = self._check_truck_route(route1)
        capacity2, uav_num2 = capacity, uav_num
        if uav_num is None:
            return None
        else:
            self.truck_capacity[truck1] = capacity
            self.truck_uav_num[truck1] = uav_num
        if truck1 != truck2:
            return None
            capacity2, uav_num2 = self._check_truck_route(route2)
            if uav_num2 is None:
                return None
            else:
                self.truck_capacity[truck2] = capacity2
                self.truck_uav_num[truck2] = uav_num2

        delet_edges = [[d], [e]]
        add_edges = [[], []]
        if len(route1) > s_index + 2:  # route1已经插入了无人机点
            delet_edges[0].append(s)
            delet_edges[1].append(route1[s_index + 2])
            add_edges[0].append(d)
            add_edges[1].append(route1[s_index + 2])
        else:  # s为当前车辆最后一点
            delet_edges[0].append(s)
            delet_edges[1].append(0)
            add_edges[0].append(d)
            add_edges[1].append(0)
        delet_edge_id = self.solution_graph.edge_ids(delet_edges[0], delet_edges[1])
        self.solution_graph = dgl.remove_edges(self.solution_graph, delet_edge_id)
        self.solution_graph = dgl.add_edges(self.solution_graph, add_edges[0], add_edges[1])
        self.uav_node, self.uav_take_off, self.uav_land = self._check_uav_circle(self.solution_graph, uav_node,
                                                                                 uav_take_off, uav_land)
        if self.uav_node is False:
            return None
        return self.cal_cost()

    def uav_node_change_(self, node1, node2):  # move 形式: (node1, node2) 可交换,无人机的起降点为node1
        """变更无人机的起飞，或者降落点
           node1: 无人机的起飞或降落点
           node2： 待交换的点=>可与node1同为降落点或同为起飞点"""
        # assert node2 not in self.uav_take_off.keys()
        # assert node2 not in self.uav_land.keys()
        node1_truck = self.node_truck[node1]
        node2_truck = self.node_truck[node2]
        node1_index = self.truck_route[node1_truck].index(node1)
        node2_index = self.truck_route[node2_truck].index(node2)
        route1 = self.truck_route[node1_truck]  # 卡车路径不变
        route2 = self.truck_route[node2_truck]  # 卡车路径不变
        if node1_truck != node2_truck:
            return None
        if node1 in self.uav_take_off.keys():
            assert node2 not in self.uav_land.keys()
            d, e = self.uav_take_off[node1]
            if not self.judge_fli_range(node2, d, e):
                return None
            if node2 in self.uav_take_off.keys():
                d2, e2 = self.uav_take_off[node2]
                if not self.judge_fli_range(node1, d2, e2):
                    return None
            uav_node = self.uav_node
            uav_take_off = self.uav_take_off
            uav_land = self.uav_land
            del uav_take_off[node1]
            del uav_node[d]
            del uav_land[e]
            delet_edges = [[node1], [d]]
            add_edges = [[node2], [d]]

            if node2 in self.uav_take_off.keys():
                d2, e2 = self.uav_take_off[node2]
                del uav_take_off[node2]
                del uav_node[d2]
                del uav_land[e2]
                uav_take_off[node1] = (d2, e2)
                uav_node[d2] = (node1, e2)
                uav_land[e2] = (node1, d2)
                delet_edges[0].append(node2)
                delet_edges[1].append(d2)
                add_edges[0].append(node1)
                add_edges[1].append(d2)
            uav_take_off[node2] = (d, e)
            uav_node[d] = (node2, e)
            uav_land[e] = (node2, d)

        elif node1 in self.uav_land.keys():
            assert node2 not in self.uav_take_off.keys()
            s, d = self.uav_land[node1]
            if not self.judge_fli_range(s, d, node2):
                return None
            if node2 in self.uav_land.keys():
                s2, d2 = self.uav_land[node2]
                if not self.judge_fli_range(s2, d2, node1):
                    return None
            uav_node = self.uav_node
            uav_take_off = self.uav_take_off
            uav_land = self.uav_land
            del uav_take_off[s]
            del uav_node[d]
            del uav_land[node1]
            delet_edges = [[d], [node1]]
            add_edges = [[d], [node2]]

            if node2 in self.uav_land.keys():
                s2, d2 = self.uav_land[node2]
                del uav_take_off[s2]
                del uav_node[d2]
                del uav_land[node2]
                uav_take_off[s2] = (d2, node1)
                uav_node[d2] = (s2, node1)
                uav_land[node1] = (s2, d2)
                delet_edges[0].append(d2)
                delet_edges[1].append(node2)
                add_edges[0].append(d2)
                add_edges[1].append(node1)
            uav_take_off[s] = (d, node2)
            uav_node[d] = (s, node2)
            uav_land[node2] = (s, d)
        else:
            print(r"uav_node_change出错！！！")
            return None

        capacity, uav_num = self._check_truck_route(route1)
        capacity2, uav_num2 = capacity, uav_num
        if capacity is None:  # node1 车辆路径约束
            return None
        else:
            self.truck_capacity[node1_truck] = capacity
            self.truck_uav_num[node1_truck] = uav_num

        # 变化图
        delet_edge_id = self.solution_graph.edge_ids(delet_edges[0], delet_edges[1])
        self.solution_graph = dgl.remove_edges(self.solution_graph, delet_edge_id)
        self.solution_graph = dgl.add_edges(self.solution_graph, add_edges[0], add_edges[1])
        self.uav_node, self.uav_take_off, self.uav_land = self._check_uav_circle(self.solution_graph, uav_node,
                                                                                 uav_take_off, uav_land)
        if self.uav_node is False:
            return None
        return self.cal_cost()

    def uav_node_exchange_(self, node1, node2):
        s1, e1 = self.uav_node[node1]
        s2, e2 = self.uav_node[node2]
        if not self.judge_fli_range(s1, node2, e1):
            return None
        if not self.judge_fli_range(s2, node1, e2):
            return None
        del self.uav_node[node1]
        del self.uav_take_off[s1]
        del self.uav_land[e1]
        del self.uav_node[node2]
        del self.uav_take_off[s2]
        del self.uav_land[e2]
        self.uav_node[node1] = (s2, e2)
        self.uav_take_off[s2] = (node1, e2)
        self.uav_land[e2] = (s2, node1)
        self.uav_node[node2] = (s1, e1)
        self.uav_take_off[s1] = (node2, e1)
        self.uav_land[e1] = (s1, node2)

        delet_edges = [[s1, node1, s2, node2], [node1, e1, node2, e2]]
        add_edges = [[s1, node2, s2, node1], [node2, e1, node1, e2]]

        delet_edge_id = self.solution_graph.edge_ids(delet_edges[0], delet_edges[1])
        self.solution_graph = dgl.remove_edges(self.solution_graph, delet_edge_id)
        self.solution_graph = dgl.add_edges(self.solution_graph, add_edges[0], add_edges[1])
        self.uav_node, self.uav_take_off, self.uav_land = \
            self._check_uav_circle(self.solution_graph, self.uav_node, self.uav_take_off, self.uav_land)
        if self.uav_node is False:
            return None
        return self.cal_cost()

    def uav_truck_exchange_(self, node1, node2):
        """
        :param node1: 无人机点
        :param node2: 卡车点
        :return:
        """
        if node1 not in self.uav_node:
            a = node1
            node1 = node2
            node2 = a
        s, e = self.uav_node[node1]
        if node2 in self.uav_take_off.keys() or node2 in self.uav_land.keys():
            return None
        if not self.judge_fli_range(s, node2, e):
            return None
        truck = self.node_truck[node2]
        i = self.truck_route[truck].index(node2)
        # 变更图
        delet_edges = [[s, node1, self.truck_route[truck][i - 1]], [node1, e, node2]]
        add_edges = [[s, node2, self.truck_route[truck][i - 1]], [node2, e, node1]]
        if i + 1 < len(self.truck_route[truck]):
            delet_edges[0].append(node2)
            delet_edges[1].append(self.truck_route[truck][i + 1])
            add_edges[0].append(node1)
            add_edges[1].append(self.truck_route[truck][i + 1])
        else:
            delet_edges[0].append(node2)
            delet_edges[1].append(0)
            add_edges[0].append(node1)
            add_edges[1].append(0)

        # 更改
        self.truck_route[truck][i] = node1
        del self.node_truck[node2]
        self.node_truck[node1] = truck
        del self.uav_node[node1]
        self.uav_node[node2] = (s, e)  # d: s, e
        self.uav_take_off[s] = (node2, e)  # s: (d, e)
        self.uav_land[e] = (s, node2)  # e: (s, d)

        delet_edge_id = self.solution_graph.edge_ids(delet_edges[0], delet_edges[1])
        self.solution_graph = dgl.remove_edges(self.solution_graph, delet_edge_id)
        self.solution_graph = dgl.add_edges(self.solution_graph, add_edges[0], add_edges[1])
        self.uav_node, self.uav_take_off, self.uav_land = \
            self._check_uav_circle(self.solution_graph, self.uav_node, self.uav_take_off, self.uav_land)
        if self.uav_node is False:
            return None
        return self.cal_cost()

    def _check_truck_route(self, route):
        """检查路径的货物容量约束和无人机数量约束,
        返回新的capacity数组和uav_num数组"""
        capacity = [self.route_map.total_truck_capacity]
        uav_num = [self.route_map.uav_num]
        for node in route:
            if node == 0:
                continue
            current_cap = capacity[-1] - self.route_map.demand[node]
            current_uav_num = uav_num[-1]
            if node in self.uav_take_off.keys():
                uav_node = self.uav_take_off[node][0]
                current_cap -= self.route_map.demand[uav_node]
                current_uav_num -= 1
            if current_cap < 0:
                return None, None
            if node in self.uav_land.keys():
                current_uav_num += 1
            if current_uav_num < 0 or current_uav_num > self.route_map.max_uav_num:  # 卡车的无人机容量大于已有无人机
                return None, None
            capacity.append(current_cap)
            uav_num.append(current_uav_num)
        return capacity, uav_num

    def _check_uav_circle(self, s_graph, uav_node=None, uav_take_off=None, uav_land=None):
        """
        表示解的图终若存在圈，则无人机的起降方向需改变
        改变方法：将圈中的无人机路径全部调换顺序
        :param graph:
        :return:
        """
        delet_edges = s_graph.in_edges(0)
        delet_edges = s_graph.edge_ids(delet_edges[0], delet_edges[1])
        assert len(delet_edges) == self.route_map.truck_num
        s_graph = dgl.remove_edges(s_graph, delet_edges)
        net_graph = s_graph.to_networkx()
        circles = list(nx.simple_cycles(net_graph))
        circles = sorted(circles, key=lambda x: len(x))

        if uav_node is None:
            if len(circles) == 0:
                return True, True, True
            else:
                return False, False, False
        # 将圈中的无人机路径全部调换顺序
        node_set = set([])
        for c in circles:
            for node in c:
                delet_edges = [[], []]
                if node in uav_node.keys() and node not in node_set:
                    s, e = uav_node[node]
                    s_truck = self.node_truck[s]
                    e_truck = self.node_truck[e]
                    s_index = self.truck_route[s_truck].index(s)
                    e_index = self.truck_route[e_truck].index(e)
                    if s_truck == e_truck and s_index < e_index:
                        continue
                    node_set.add(node)
                    s, e = uav_node[node]
                    uav_node[node] = (e, s)
                    del uav_take_off[s]
                    uav_take_off[e] = (node, s)
                    del uav_land[e]
                    uav_land[s] = (e, node)
                    # 变图
                    delet_edges[0].append(s)
                    delet_edges[1].append(node)
                    delet_edges[0].append(node)
                    delet_edges[1].append(e)
                    delet_edges_id = self.solution_graph.edge_ids(delet_edges[0], delet_edges[1])
                    self.solution_graph = dgl.remove_edges(self.solution_graph, delet_edges_id)
                    self.solution_graph = dgl.add_edges(self.solution_graph, delet_edges[1], delet_edges[0])

                    capacity, uav_num = self._check_truck_route(self.truck_route[s_truck])
                    if capacity is None:  # node1 车辆路径约束
                        return False, False, False
                    else:
                        self.truck_capacity[s_truck] = capacity
                        self.truck_uav_num[s_truck] = uav_num
                    if s_truck != e_truck:
                        capacity2, uav_num2 = self._check_truck_route(self.truck_route[e_truck])
                        if capacity2 is None:  # node1 车辆路径约束
                            return False, False, False
                        else:
                            self.truck_capacity[e_truck] = capacity2
                            self.truck_uav_num[e_truck] = uav_num2
                    # 检查是否还有圈存在
                    P, _, _ = self._check_uav_circle(self.solution_graph)
                    if P:
                        return uav_node, uav_take_off, uav_land
                    else:
                        return False, False, False

        return uav_node, uav_take_off, uav_land


    def print_solution(self):
        print("cost: \n %s" % self.total_cost)
        print("over_time: \n %s" % self.total_over_time)
        print("erve_time: \n %s" % self.total_serve_time)
        print("truck_route: \n %s" % self.truck_route)
        print("node_truck: \n %s" % self.node_truck)
        print("truck_capacity: \n %s" % self.truck_capacity)
        print("truck_uav_num: \n %s" % self.truck_uav_num)
        print("uav_take_off: \n %s" % self.uav_take_off)
        print("uav_land: \n %s" % self.uav_land)
        print("arive_time: \n %s" % self.solution_graph.ndata['a_time'])
        print("leave_time: \n %s" % self.solution_graph.ndata['l_time'])
        print("serve_time: \n %s" % self.solution_graph.ndata['s_time'])
        print("require_time: \n %s" % self.solution_graph.ndata['dead_time'])

    def print_graph(self):
        s_graph = self.solution_graph
        delet_edges = s_graph.in_edges(0)
        delet_edges = s_graph.edge_ids(delet_edges[0], delet_edges[1])
        assert len(delet_edges) == self.route_map.truck_num
        s_graph = dgl.remove_edges(s_graph, delet_edges)
        net_graph = s_graph.to_networkx()
        plt.figure(figsize=(10, 6))
        plt.subplot(122)
        plt.title('Directed graph ,DGL', fontsize=20)
        nx.draw(net_graph, with_labels=True)
        plt.show()
