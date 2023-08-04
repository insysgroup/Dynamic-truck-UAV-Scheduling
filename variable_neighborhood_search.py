# -*- coding: utf-8 -*-
import copy
import random
import time
from collections import deque
from env.route_map import RouteMap
from env.solution import Solution
import pandas as pd
import winsound
import math


class VNS:
    def __init__(self, route_map: RouteMap, initial_s: Solution):
        # 算法参数
        self.epoch_num = 1000
        self.tabu_length = 25
        self.max_neighbor_size = 1000  # self.tabu_length + 1

        # 搜索变量
        self.route_map = route_map
        self.current_s = initial_s
        self.best_s = self.copy_(initial_s)
        self.best_cost = self.best_s.cal_cost()
        self.tabu_list = deque([])

    def copy_(self, solution):
        new_solution = Solution(self.route_map)
        new_solution.total_cost = solution.total_cost
        new_solution.total_over_time = solution.total_over_time
        new_solution.total_serve_time = solution.total_serve_time
        new_solution.solution_graph = copy.deepcopy(solution.solution_graph)
        new_solution.truck_route = copy.deepcopy(solution.truck_route)  # k: []
        new_solution.node_truck = copy.deepcopy(solution.node_truck)  # node_id: truck_id
        new_solution.truck_capacity = copy.deepcopy(solution.truck_capacity)
        new_solution.truck_uav_num = copy.deepcopy(solution.truck_uav_num)
        new_solution.uav_node = copy.deepcopy(solution.uav_node)
        new_solution.uav_take_off = copy.deepcopy(solution.uav_take_off)  # s: (d, e)
        new_solution.uav_land = copy.deepcopy(solution.uav_land)
        return new_solution

    def _add_move(self, candi_list: deque, move):
        if len(candi_list) < 1:
            candi_list.append(move)
        elif move[0][2] == 2:
            candi_list.appendleft(move)
        elif candi_list[0][1] >= move[1]:
            candi_list.appendleft(move)
        elif move[1] > candi_list[-1][1]:
            if len(candi_list) >= self.max_neighbor_size:
                return candi_list
            else:
                candi_list.append(move)
        elif len(candi_list) < self.max_neighbor_size:
            for i in range(len(candi_list)):
                if move[1] < candi_list[i][1]:
                    candi_list.insert(i, move)
                    return candi_list
            candi_list.append(move)
        return candi_list

    def get_neighbor(self, sol: Solution, type=0):
        """
        :param sol:
        :param add_uav: 0: 所有领域;1：增加无人机，-1：减少无人机; 2 保持uav_node的数量
        :return:
        """
        neighbors = deque([])
        # 变卡车点
        for node1 in sol.node_truck.keys():
            # 车->无人机
            if type == 2:
                info = self.copy_(sol).truck_node_to_uav_(node1)
                if info is not None:
                    tem_cost = info
                    move = ((0, node1, 2), tem_cost)  # 1: 卡车路径交换,2: 车变无人机, 无人机便车, 3/4: 变无人起终点  ()
                    self._add_move(neighbors, move)
                    return neighbors, True
            if type == 1:
                for node2 in sol.node_truck.keys():
                    if node1 >= node2:
                        continue
                    # 卡车交换
                    info = self.copy_(sol).truck_node_exchange_(node1, node2)
                    if info is not None:

                        tem_cost = info
                        if node1 < node2:
                            move = ((node1, node2, 1), tem_cost)  # 1: 卡车路径交换,2: 车变无人机, 无人机便车, 3/4: 变无人起终点  ()
                        else:
                            move = ((node2, node1, 1), tem_cost)  # (node2, node1, 1)  小点在前,便于查找禁忌
                        self._add_move(neighbors, move)
        # 变无人机
        for s, de in sol.uav_take_off.items():
            # 无人机中间点交换
            if type == 5:
                for node in sol.uav_node.keys():
                    if node > de[0]:
                        info = self.copy_(sol).uav_node_exchange_(de[0], node)
                        if info is not None:
                            tem_cost = info
                            move = ((de[0], node, 5), tem_cost)
                            self._add_move(neighbors, move)
            # 无人机起降点变
            for node in sol.node_truck.keys():
                # 起点交换
                if type == 3:
                    if node not in sol.uav_land.keys():  # node 可为起飞点或空余点
                        if node in sol.uav_take_off.keys() and node <= s:
                            continue
                        info = self.copy_(sol).uav_node_change_(s, node)
                        if info is not None:
                            tem_cost = info
                            if s < node:
                                move = ((s, node, 3), tem_cost)
                            else:
                                move = ((node, s, 3), tem_cost)
                            self._add_move(neighbors, move)
                # 终点交换
                if type == 4:
                    if node not in sol.uav_take_off.keys():  # node 可为降落点或空余点
                        if node in sol.uav_land.keys() and node <= de[1]:
                            continue
                        info = self.copy_(sol).uav_node_change_(de[1], node)
                        if info is not None:
                            tem_cost = info
                            if de[1] < node:
                                move = ((de[1], node, 4), tem_cost)
                            else:
                                move = ((node, de[1], 4), tem_cost)
                            self._add_move(neighbors, move)
                # 服务点与货车交换
                if type == 6:
                    if node not in sol.uav_take_off.keys() and node not in sol.uav_land.keys():
                        info = self.copy_(sol).uav_truck_exchange_(de[0], node)
                        if info is not None:
                            tem_cost = info
                            if de[0] < node:
                                move = ((de[0], node, 6), tem_cost)
                            else:
                                move = ((node, de[0], 6), tem_cost)
                            self._add_move(neighbors, move)
        return neighbors, False

    def choose_move(self, neighbors):
        """
        1:卡车点交换
        2:车变无人机,
        3:无人机起点换卡车
        4:无人机终点换卡车
        5:无人机服务点换
        6:无人机访问点和卡车服务点换
        """
        moves = []
        trucks = set()
        uavs = set()  # 涉及无人机路线的点
        move_type = random.randint(1, 6)
        assert len(neighbors) >= 1
        for move in neighbors:
            if move[0][2] == 2:
                moves.append(move)
                break
            if move[1] >= self.current_s.total_cost:
                break
            elif len(trucks) >= self.route_map.truck_num:
                break

            if move[0][2] == 1:  #   in [1, 3, 4]:
                truck1 = self.current_s.node_truck[move[0][0]]
                truck2 = self.current_s.node_truck[move[0][1]]
                if truck1 not in trucks and truck2 not in trucks:
                    moves.append(move)
                    trucks.add(truck1)
                    trucks.add(truck2)
            elif move[0][2] == 3:
                truck1 = self.current_s.node_truck[move[0][0]]
                truck2 = self.current_s.node_truck[move[0][1]]
                if move[0][0] in self.current_s.uav_take_off.keys():
                    uav_node = self.current_s.uav_take_off[move[0][0]][0]
                else:
                    uav_node = self.current_s.uav_take_off[move[0][1]][0]
                if truck1 not in trucks and truck2 not in trucks:
                    if uav_node not in uavs:
                        moves.append(move)
                        trucks.add(truck1)
                        trucks.add(truck2)
                        uavs.add(uav_node)
            elif move[0][2] == 4:
                truck1 = self.current_s.node_truck[move[0][0]]
                truck2 = self.current_s.node_truck[move[0][1]]
                if move[0][0] in self.current_s.uav_land.keys():
                    uav_node = self.current_s.uav_land[move[0][0]][1]
                else:
                    uav_node = self.current_s.uav_land[move[0][1]][1]
                if truck1 not in trucks and truck2 not in trucks:
                    if uav_node not in uavs:
                        moves.append(move)
                        trucks.add(truck1)
                        trucks.add(truck2)
                        uavs.add(uav_node)
            elif move[0][2] == 5:
                s1, e1 = self.current_s.uav_node[move[0][0]]
                s2, e2 = self.current_s.uav_node[move[0][1]]
                truck1 = self.current_s.node_truck[s1]
                truck2 = self.current_s.node_truck[s2]
                truck3 = self.current_s.node_truck[e1]
                truck4 = self.current_s.node_truck[e2]
                if (truck1 not in trucks and truck2 not in trucks) \
                        and (truck3 not in trucks and truck4 not in trucks):
                    if move[0][0] not in uavs and move[0][1] not in uavs:
                        moves.append(move)
                        trucks.add(truck1)
                        trucks.add(truck2)
                        trucks.add(truck3)
                        trucks.add(truck4)
                        uavs.add(move[0][0])
                        uavs.add(move[0][1])
            else:  # 类型 2, 6
                truck_node = move[0][1]
                uav_node = move[0][0]
                if move[0][1] not in self.current_s.node_truck.keys():
                    truck_node = move[0][0]
                    uav_node = move[0][1]
                truck1 = self.current_s.node_truck[truck_node]
                if truck1 not in trucks and (uav_node not in uavs or uav_node == 0):
                    # if truck_node not in uavs or uav_node == 0:
                    moves.append(move)
                    trucks.add(truck1)
                    uavs.add(uav_node)
                    uavs.add(truck_node)
        return moves

    def carry_move(self, move, sol: Solution):
        if move[0][2] == 1:  # truck_node_exchange
            tem_sol = self.copy_(sol)
            info = tem_sol.truck_node_exchange_(move[0][0], move[0][1])
            if info is None:
                print("truck_node_exchange_1 错误!!!!!")
                return None
            else:
                sol = tem_sol
            return sol
        elif move[0][2] == 2:  # truck_node_to_uav
            assert move[0][0] == 0
            if move[0][1] in sol.node_truck.keys():
                tem_sol = self.copy_(sol)
                info = tem_sol.truck_node_to_uav_(move[0][1])
                if info is None:
                    print("truck_node_to_uav2_1 错误!!!!!")
                    return None
                else:
                    sol = tem_sol
            else:  # uav to truck
                tem_sol = self.copy_(sol)
                s, e = tem_sol.uav_node[move[0][1]]
                info = tem_sol.uav_node_to_truck_((s, move[0][1], e))
                if info is None:
                    print("uav_node_to_truck2_2 错误!!!!!")
                    return None
                else:
                    sol = tem_sol
            return sol
        elif move[0][2] == 3:  # uav_node_change 无人机起点交换
            if move[0][0] in sol.uav_take_off.keys():
                tem_sol = self.copy_(sol)
                info = tem_sol.uav_node_change_(move[0][0], move[0][1])
                if info is None:
                    print("uav_node_change3_1 错误!!!!!")
                    return None
                else:
                    sol = tem_sol
            else:
                tem_sol = self.copy_(sol)
                info = tem_sol.uav_node_change_(move[0][1], move[0][0])
                if info is None:
                    print("uav_node_change3_2 错误!!!!!")
                    return None
                else:
                    sol = tem_sol
            return sol
        elif move[0][2] == 4:  # uav_node_change 无人机终点交换
            if move[0][0] in sol.uav_land.keys():
                tem_sol = self.copy_(sol)
                info = tem_sol.uav_node_change_(move[0][0], move[0][1])
                if info is None:
                    print("uav_node_change4_1 错误!!!!!")
                    return None
                else:
                    sol = tem_sol
            else:
                tem_sol = self.copy_(sol)
                info = tem_sol.uav_node_change_(move[0][1], move[0][0])
                if info is None:
                    print("uav_node_change4_2 错误!!!!!")
                    return None
                else:
                    sol = tem_sol
            return sol
        elif move[0][2] == 5:  # 无人机服务点交换
            tem_sol = self.copy_(sol)
            info = tem_sol.uav_node_exchange_(move[0][0], move[0][1])
            if info is None:
                print("truck_node_exchange5 错误!!!!!")
                return None
            else:
                sol = tem_sol
            return sol
        elif move[0][2] == 6:  # 无人机服务点与货车点减缓
            tem_sol = self.copy_(sol)
            info = tem_sol.uav_truck_exchange_(move[0][0], move[0][1])
            if info is None:
                print("truck_node_exchange6 错误!!!!!")
                return None
            else:
                sol = tem_sol
            return sol
        else:
            print("错误!carry_move错误!!!!!")
            return None

    def search(self):
        df1 = pd.DataFrame(columns=["demand", "iteration", "best_cost", "calcu_time"])
        self.current_s.print_solution()
        self.tabu_list = deque([])
        no_better = 0
        inter = 0
        k_iter = 1
        while inter < self.epoch_num:
            neighbors, add_uav = self.get_neighbor(self.current_s, type=k_iter)
            while len(neighbors) < 1:
                k_iter += 1
                if k_iter > 6:
                    k_iter = 1
                neighbors, add_uav = self.get_neighbor(self.current_s, type=k_iter)
            random_move = random.choice(neighbors)
            tem_s = self.carry_move(random_move, self.current_s)
            if tem_s is not None:
                self.current_s = tem_s
            # VNS
            neighbor_type = 1
            k_iter += 1
            if k_iter > 6:
                k_iter = 1
            while neighbor_type <= 6:
                print(f'{inter} iteration: neighbor_type {neighbor_type}, cur_cost = {self.current_s.total_cost:.5f},best_cost = {self.best_cost:.5f}, nobetter={no_better}')
                neighbors, add_uav = self.get_neighbor(self.current_s, type=neighbor_type)
                df1.loc[len(df1.index)] = [len(instance.demand) - 1, inter, self.best_cost, time.time() - t_start]
                print(f'neghbor_size = {len(neighbors)}')
                if len(neighbors) < 1:
                    neighbor_type += 1
                    continue
                if add_uav:
                    moves = [move for move in neighbors if move[0][2] == 2]
                else:
                    moves = self.choose_move(neighbors)
                print(f'moves = {moves}')
                if len(moves) < 1:
                    neighbor_type += 1
                    continue
                for move in moves:
                    tem_s = self.carry_move(move, self.current_s)
                    neighbor_type = 1
                    if tem_s is not None:
                        self.current_s = tem_s
                        if self.current_s.total_cost < self.best_cost:
                            no_better = 0
                            self.best_s = self.copy_(self.current_s)
                            self.best_cost = self.current_s.total_cost
                inter += 1
                print()
                no_better += 1
                if no_better > 200:
                    break
            if no_better > 200:
                break
        df1.to_csv('VNS-动态飞行伙伴-需求-150-迭代数据.csv')
        return self.best_s


if __name__ == "__main__":
    df = pd.DataFrame(columns=["demand", "cost", "waiting time", "calcu_time"])
    for item in range(1, 11):
        t_start = time.time()
        instance = RouteMap(item)
        initial_sol = Solution(instance)
        initial_sol.init_routes_2(seed=1)
        tabu_algrothim = VNS(instance, initial_sol)
        final_sol = tabu_algrothim.search()
        t_end = time.time()
        t_continue = t_end - t_start
        df.loc[len(df.index)] = [len(instance.demand) - 1, final_sol.total_cost,
                                 math.floor(final_sol.total_cost / 10000), t_continue]
    df.to_csv('VNS-动态飞行伙伴-需求-150.csv')