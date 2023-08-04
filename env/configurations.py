seed = 2
M = 1000
uav_num = 2
max_uav_num = 3
v_k = 40  # 货车速度
total_truck_capacity = 4500

# uav information
node_num = 100
v_u = 60  # 无人机速度
fli_range = 10

truck_serve_node = 10
if node_num < 20:
    truck_num = 2
else:
    truck_num = int(node_num / truck_serve_node)

total_uav_num = uav_num * truck_num

# demand information
max_require_load = 10  # 每个点最大需要的物资量
need_time = 80  # 最晚的需求产生时间
lamda = 5
distribution = "uniform"  # uniform-均匀分布/ cluster-聚类分布
# distribution = "cluster"
cluster_num = 3
cluster_step = 1.0
