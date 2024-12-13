
import os, sys

if 'SUMO_HOME' in os.environ:  # 检查SUMO_HOME是否在系统环境变量中，进行下一步仿真的必要条件
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')

    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
import matplotlib.pyplot as plt

plt.rcParams['font.sans-serif'] = ['SimHei']
plt.rcParams['axes.unicode_minus'] = False

import traci
import sumolib
import networkx as nx
import pickle
from itertools import islice


def save_network():
    G = nx.DiGraph()
    # 将网络中的每个连接添加到图中
    for edge in netData.getEdges():
        from_id = edge.getID()
        for to_edge in edge.getOutgoing():
            to_id = to_edge.getID()
            length = edge.getLength()  # 路段长度
            max_speed = edge.getSpeed()  # 最大限速
            if max_speed > 0:  # 避免除以零
                travel_time = length / max_speed  # 计算通行时间
            else:
                travel_time = float('inf')  # 无限大的通行时间表示此路不可通行
            G.add_edge(from_id, to_id, weight=travel_time)
    # 将图序列化到文件
    with open("network_route.pkl", 'wb') as f:
        pickle.dump(G, f)
    return G

graph_path = r'D:\intereting\DynamicNavigation\findPath\findPath\network_route.pkl'
def load_network():
    # 从文件加载图
    with open(graph_path, 'rb') as f:
        G = pickle.load(f)
    return G


def find_k_shortest_paths(G, source, target, k):
    # 查找最短的k条路径

    return list(islice(nx.shortest_simple_paths(G, source, target, weight='weight'), k))
    # return list(nx.shortest_simple_paths(G, source, target, weight='weight'))[:k]


def calculate_path_travel_time(G, path):
    # 计算路径的总通行时间
    total_time = 0.0
    for i in range(len(path) - 1):
        total_time += G[path[i]][path[i + 1]]['weight']
    return total_time

def get_shortest_path(start_edge_id, end_edge_id):
    #参数里少了net_path
    #计算理论最短路径并得到途径路段
    # net = sumolib.net.readNet(net_path)
    start_edge = netData.getEdge(start_edge_id)
    end_edge = netData.getEdge(end_edge_id)
    shortest_path = netData.getShortestPath(start_edge, end_edge)[0]
    
    # 从最短路径中提取路径ID
    shortest_edge_list = [edge.getID() for edge in shortest_path]
    # shortest_node_list = [start_edge.getFromNode().getID()]
    # for edge in shortest_path:
    #     shortest_node_list.append(edge.getToNode().getID())
    return shortest_path, shortest_edge_list#,shortest_node_list


def run():
    sumoBinary = sumo_path + "sumo-gui"  # -gui就是打开gui界面
    sumoCmd = [sumoBinary, "-n", net_file, "-r", rou_file,"-c", cfg_file]
    traci.start(sumoCmd)
    step = 0
    # 经过的路段
    pass_edges = []
    # 经过的路口
    pass_node = []
    # 两点之间的路径
    # route = traci.simulation.findRoute(start_edge, end_edge)
    shortest_path, shortest_edge_list= get_shortest_path(start_edge, end_edge)
    v_id = "v1"

    # if route is not None:
    #     route_id = "random_route"
    #     traci.route.add(route_id, route.edges)
    #     traci.vehicle.add(v_id, route_id)
    if shortest_path is not None:
        route_id = "random_route"
        traci.route.add(route_id, shortest_edge_list)
        traci.vehicle.add(v_id, route_id)
    vehicle_ids = []
    for i in range(50):
        vehicle_id = f"veh_{i}"  # 生成车辆ID
        route_id = f"route_{i}"  # 生成路线ID
        traci.route.add(route_id, shortest_edge_list)  # 添加路线
        traci.vehicle.add(vehicle_id, route_id)
        vehicle_ids.append(vehicle_id) 
    step = 0
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        step += 1
        sim_time = traci.simulation.getTime()
        edge_list = [f"edge_{i}" for i in range(1, 51)]
        if v_id in traci.vehicle.getIDList():
            traci.gui.setZoom('View #0', 1000)#8017.95
            #设置视角跟随车辆v1
            traci.gui.trackVehicle('View #0', v_id)
            # 获取车辆所处车道
            # edge_ = traci.vehicle.getLaneID(v_id).split("_")[0]
        for vechicle_id in vehicle_ids:
            if vechicle_id in traci.vehicle.getIDList():
                # traci.simulation.pause()
                i = vehicle_ids.index(vechicle_id) 
                edge_list[i]=traci.vehicle.getLaneID(vechicle_id).split("_")[0]
                print(f"{vechicle_id}当前所在车道：{edge_list[i]}")
                edge_ = edge_list[i]
                if str(edge_).startswith(":"):
                    pass
                else:
                    edge_node = netData.getEdge(edge_).getToNode()
                    # 获取路口属性
                    nod_id = edge_node.getID()
                    # node_type = edge_node.getType()
                    # 记录经过的路段
                    if edge_ in pass_edges:
                        pass
                    else:
                        pass_edges.append(edge_)
                        if nod_id in pass_node:
                                # 获取前三条最短路径
                            # traci.simulation.pause()
                            k_shortest_paths = find_k_shortest_paths(G, edge_, end_edge, 3)
                                # # 打印结果和通行时间
                            for i, path in enumerate(k_shortest_paths, 1):
                                travel_time = calculate_path_travel_time(G, path)
                                print(f"当前路口{nod_id}路径 {i}: {path}")

                                if i == 1:
                                    traci.route.add(str(sim_time), path)
                                    # traci.vehicle.add("vid_{}".format(sim_time), str(sim_time))

                            print(f"路径 {i}:  行程时间: {travel_time:.2f} seconds")
                            # traci.simulation.resume()
                        else:
                            pass_node.append(nod_id)
            
    traci.close()
import random
random.seed(88)

# 程序入口
if __name__ == '__main__':
    sumo_path = os.path.join(os.environ['SUMO_HOME'], 'bin') + "/"
    net_file = r'D:\intereting\DynamicNavigation\findPath\findPath\ingolstadt21.net.xml'  # 路网文件
    rou_file = r'D:\intereting\DynamicNavigation\findPath\findPath\ingolstadt21.rou.xml'  # 路网文件
    cfg_file = r'D:\intereting\DynamicNavigation\findPath\findPath\ingolstadt21.sumocfg'
    netData = sumolib.net.readNet(net_file)  # 读取路网文件
    edges = netData.getEdges()
    edge_ids = [edge.getID() for edge in edges]
    # start_edge = random.choice(edge_ids) 
    # end_edge = random.choice(edge_ids)
    start_edge = "-30482615#4"
    end_edge = "-315358244"
    edges = netData.getEdges()
    # 加载网络
    # Grah = save_network()
    G = load_network()
    # 开始运行仿真
    run()

# 程序入口
# if __name__ == '__main__':
#     sumo_path = os.path.join(os.environ['SUMO_HOME'], 'bin') + "/"
#     net_file = r'D:\intereting\DynamicNavigation\findPath\findPath\test_data\test.net.xml'  # 路网文件
#     rou_file = r'D:\intereting\DynamicNavigation\findPath\findPath\test_data\test.rou.xml'  # 路网文件
#     cfg_file = r'D:\intereting\DynamicNavigation\findPath\findPath\ingolstadt21.sumocfg'
#     netData = sumolib.net.readNet(net_file)  # 读取路网文件
#     start_edge = "right0D0"
#     end_edge = "A2left2"
#     edges = netData.getEdges()
#     # 加载网络
#     # Grah = save_network()
#     G = load_network()
#     # 开始运行仿真
#     run()
