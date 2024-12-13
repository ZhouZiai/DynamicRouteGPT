import os,sys

if 'SUMO_HOME' in os.environ:  # 检查SUMO_HOME是否在系统环境变量中，进行下一步仿真的必要条件
    tools = os.path.join(os.environ['SUMO_HOME'],'tools')

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
            G.add_edge(from_id,to_id,weight=travel_time)
    # 将图序列化到文件
    with open("hangzhou_route.pkl",'wb') as f:
        pickle.dump(G,f)
    return G

# graph_path = r'D:\intereting\DynamicNavigation\findPath\findPath\network_route.pkl'
#gridpath
# graph_path = r'D:\intereting\DynamicNavigation\findPath\findPath\test_data\孙倩\grid4x4\network_grid.pkl'
#Hangzhou
graph_path = r'D:\intereting\DynamicNavigation\findPath\findPath\test_data\孙倩\Hangzhou-4x4\hangzhou_route.pkl'

def load_network():
    # 从文件加载图
    with open(graph_path,'rb') as f:
        G = pickle.load(f)
    return G


def find_k_shortest_paths(G,source,target,k):
    # 查找最短的k条路径

    return list(islice(nx.shortest_simple_paths(G,source,target,weight='weight'),k))
    # return list(nx.shortest_simple_paths(G,source,target,weight='weight'))[:k]


def calculate_path_travel_time(G,path):
    # 计算路径的总通行时间
    total_time = 0.0
    for i in range(len(path) - 1):
        total_time += G[path[i]][path[i + 1]]['weight']
    return total_time

def get_shortest_path(start_edge_id,end_edge_id):
    #参数里少了net_path
    #计算理论最短路径并得到途径路段
    # net = sumolib.net.readNet(net_path)
    start_edge = netData.getEdge(start_edge_id)
    end_edge = netData.getEdge(end_edge_id)
    shortest_path = netData.getShortestPath(start_edge,end_edge)[0]
    
    # 从最短路径中提取路径ID
    shortest_edge_list = [edge.getID() for edge in shortest_path]
    # shortest_node_list = [start_edge.getFromNode().getID()]
    # for edge in shortest_path:
    #     shortest_node_list.append(edge.getToNode().getID())
    return shortest_path,shortest_edge_list#,shortest_node_list


def run():
    sumoBinary = sumo_path + "sumo-gui"  # -gui就是打开gui界面
    sumoCmd = [sumoBinary,"-n",net_file,"-r",rou_file,"-c",cfg_file]
    traci.start(sumoCmd)
    step = 0
    # 经过的路段
    pass_edges = []
    # 经过的路口
    pass_node = []
    # 两点之间的路径
    vehicle_ids =[]
    all_shortest_paths = []
    for i in range(car_num):
        vehicle_id = f"veh_{i}"
        route_id = f"route_{i}"
        start_edge = start_edges[i]
        end_edge = end_edges[i]
        shortest_path = traci.simulation.findRoute(start_edge,end_edge)
        shortest_edge_list = shortest_path.edges
        traci.route.add(route_id,shortest_edge_list)
        traci.vehicle.add(vehicle_id,route_id,depart=depart_times[i],departSpeed=10)
        vehicle_ids.append(vehicle_id)
        all_shortest_paths.append(shortest_edge_list)
    step = 0
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        step += 1
        sim_time = traci.simulation.getTime()
        edge_list = [f"edge_{i}" for i in range(1, car_num+1)]
        for vechicle_id in vehicle_ids:
            sate = vehicle_ids.index(vechicle_id)
            if vechicle_id in traci.vehicle.getIDList():
                i= vehicle_ids.index(vechicle_id)
                lane_id = traci.vehicle.getLaneID(vechicle_id)
                parts = lane_id.split("_")
                if len(parts) > 1:
                    desired_parts = parts[:-1]
                    result = "_".join(desired_parts)
                else:
                    result = lane_id
                edge_list[i] = result
                # edge_ = traci.vehicle.getLaneID(vechicle_id).split("_")[0]
                edge_ = edge_list[i]
            # 获取所在的路口
                if str(edge_).startswith(":"):
                    pass
                else:
                    edge_node = netData.getEdge(edge_).getToNode()
                    # 获取路口属性
                    nod_id = edge_node.getID()
                    # 记录经过的路段
                    if edge_ in pass_edges:
                        pass
                    else:
                        pass_edges.append(edge_)
                        if nod_id in pass_node:
                                # 获取前三条最短路径
                            # traci.simulation.pause()
                            traci.simulationStep(0) 
                            k_shortest_paths = find_k_shortest_paths(G,edge_,end_edge,3)
                                # # 打印结果和通行时间
                            for i,path in enumerate(k_shortest_paths,1):
                                # travel_time = calculate_path_travel_time(G,path)
                                # print(f"当前路口{nod_id}路径 {i}: {path}")
                                if i == 1:
                                    traci.route.add(str(sim_time),path)
                                # print(f"路径 {i}:  行程时间: {travel_time:.2f} seconds")
                            traci.simulationStep(1)
                        else:
                            pass_node.append(nod_id)

    traci.close()

if __name__ == '__main__':
    sumo_path = os.path.join(os.environ['SUMO_HOME'],'bin') + "/"
    cfg_file=r'D:\intereting\DynamicNavigation\findPath\findPath\test_data\孙倩\Hangzhou-4x4\hangzhou.sumocfg'
    net_file= r'D:\intereting\DynamicNavigation\findPath\findPath\test_data\孙倩\Hangzhou-4x4\hangzhou.net.xml'
    rou_file=r'D:\intereting\DynamicNavigation\findPath\findPath\test_data\孙倩\Hangzhou-4x4\hangzhou.rou.xml'
    netData = sumolib.net.readNet(net_file)  # 读取路网文件
    car_num = 10
    start_edges=['road_0_4_0','road_1_5_3','road_5_4_2','road_1_0_1','road_4_0_1','road_4_0_1','road_1_5_3','road_5_4_2','road_5_2_2','road_5_4_2']
    end_edges=['road_1_2_2','road_1_3_2','road_2_4_1','road_1_2_2','road_4_2_0','road_4_1_0','road_1_4_2','road_4_4_1','road_4_3_0','road_4_4_1']
    depart_times=[28,79,118,306,306,331,433,520,556,569]
    edges = netData.getEdges()
    # 加载网络
    # Grah = save_network()
    G = load_network()
    #开始运行仿真 
    run()


