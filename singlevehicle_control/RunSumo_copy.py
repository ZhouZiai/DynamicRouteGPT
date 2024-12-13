#**************************************************************************************************
#path 路径
#**********************************************************************************************

#path 路径
#path1 3*3  有一个最大的问题，没有红绿灯
sumocfg_path1=r"D:\intereting\DynamicNavigation\data\ndn_input_data\input_1200_1km_4line\cfg.sumocfg"
net_path1 = r"D:\intereting\DynamicNavigation\data\ndn_input_data\input_1200_1km_4line\input_net.net.xml"
rou_path1 = r"D:\intereting\DynamicNavigation\data\ndn_input_data\input_1200_1km_4line\routes.rou.xml"

#path2 4*4
sumocfg_path2 = r"D:\intereting\DynamicNavigation\data\ndn_input_data\input_1200_1km_5line\cfg.sumocfg"
net_path2 = r"D:\intereting\DynamicNavigation\data\ndn_input_data\input_1200_1km_5line\input_net.net.xml"
rou_path2 = r"D:\intereting\DynamicNavigation\data\ndn_input_data\input_1200_1km_5line\routes.rou.xml"

#path easy_city
sumocfg_path3= r"D:\intereting\DynamicNavigation\data\ndn_input_data\input_250\cfg.sumocfg"
net_path3 = r"D:\intereting\DynamicNavigation\data\ndn_input_data\input_250\input_net.net.xml"
rou_path3 = r"D:\intereting\DynamicNavigation\data\ndn_input_data\input_250\routes.rou.xml"

#path easy_city
sumocfg_path4 = r"D:\intereting\DynamicNavigation\RESCO\resco_benchmark\environments\cologne8\cologne8.sumocfg"
net_path4 = r"D:\intereting\DynamicNavigation\RESCO\resco_benchmark\environments\cologne8\cologne8.net.xml"
rou_path4 = r"D:\intereting\DynamicNavigation\RESCO\resco_benchmark\environments\cologne8\cologne8.rou.xml"

#path_complicated_city
sumocfg_path5 = r"D:\intereting\DynamicNavigation\RESCO\resco_benchmark\environments\ingolstadt21\ingolstadt21.sumocfg"
net_path5 = r"D:\intereting\DynamicNavigation\RESCO\resco_benchmark\environments\ingolstadt21\ingolstadt21.net.xml"
rou_path5 = r"D:\intereting\DynamicNavigation\RESCO\resco_benchmark\environments\ingolstadt21\ingolstadt21.rou.xml"

#**************************************************************************************************
#get shortest_K_paths
#**********************************************************************************************

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

#以上函数虽然可以完成预期的效果，但是想要按照我的策略来，之前写的函数并不是没有作用，好吧，这其实是两种策略
#**************************************************************************************************
# 辅助GPT决策函数，这里需要应对的是仿真开启后的函数
#****************************************************************************************************
import xml.etree.ElementTree as ET

def count_traffic_lights(node_ids, net_path):
    #计算路径中的红绿灯个数以及返回节点ID,此处无法省略
    count = 0
    light_ids = []

    # Load the net.xml file
    tree = ET.parse(net_path)
    root = tree.getroot()

    # Search for tlLogic elements with matching IDs
    for tl_logic in root.findall(".//tlLogic"):
        if tl_logic.attrib['id'] in node_ids:
            count += 1
            light_ids.append(tl_logic.attrib['id'])

    return count, light_ids

def get_edge_lengths(edge_ids):  
    #获取路径中涉及到的路段的长度
    edge_lengths = []

    # Iterate over the given edge IDs and retrieve their lengths
    for edge_id in edge_ids:
        edge = netData.getEdge(edge_id)
        length = edge.getLength()
        edge_lengths.append(length)

    return edge_lengths

def get_shortest_path(start_edge_id, end_edge_id):
    #参数里少了net_path
    #计算理论最短路径并得到途径路段
    # net = sumolib.net.readNet(net_path)
    start_edge = netData.getEdge(start_edge_id)
    end_edge = netData.getEdge(end_edge_id)
    shortest_path = netData.getShortestPath(start_edge, end_edge)[0]
    
    # 从最短路径中提取路径ID
    edge_ids = [edge.getID() for edge in shortest_path]
    # node_ids = [start_edge.getFromNode().getID()]
    # for edge in shortest_path:
    #     node_ids.append(edge.getToNode().getID())
    return shortest_path, edge_ids#,node_ids

#不行，node_ids必须单独提出
def get_node_id(start_edge_id,shortest_path):
    start_edge = netData.getEdge(start_edge_id)    
    # 从最短路径中提取路口ID
    node_ids = [start_edge.getFromNode().getID()]
    for edge in shortest_path:
        node_ids.append(edge.getToNode().getID())
    
    return node_ids

# node_ids = get_node_id(net_path, start_edge_id, shortest_path)
# print(node_ids)
#必须有一个函数，返回路径中红绿灯的个数与ID
import xml.etree.ElementTree as ET
def count_and_return_traffic_lights(intersection_id, net_path):
    traffic_light_count = 0
    # traffic_light_ids = []

    # 解析net.xml文件
    tree = ET.parse(net_path)
    root = tree.getroot()

    # 遍历net.xml中所有的tlLogic节点（表示交通信号灯逻辑）
    for tl_logic in root.findall(".//tlLogic"):
        tl_id = tl_logic.get("id")
        node_ref = tl_logic.get("node")

        # 检查当前交通信号灯是否位于给定路口
        if node_ref == intersection_id:
            traffic_light_count += 1    
            # traffic_light_ids.append(tl_id)

    return traffic_light_count

# def get_travel_time(path):
#     #获取路径长度
#     path_lengths = get_edge_lengths(path)  #list[float]
#     #获取当前道路速度
#     expected_time = float(0)
#     avg_speeds =[]
#     for edge_id in path:
#         edge = netData.getEdge(edge_id)
#         avg_speed = traci.edge.getLastStepMeanSpeed(edge)
#         if avg_speed == 0:
#             avg_speed = traci.edge.getMaxSpeed(edge_id)
#         speed = round(avg_speed, 2)
#         avg_speeds = avg_speeds.append(speed)
#     #计算路径通行时间
#     for i in len(path):
#         expected_time = expected_time + path_lengths[i] / avg_speeds[i]
# #     return expected_time
# def get_travel_time(path):
#     # 获取路径长度
#     path_lengths = get_edge_lengths(path)  # list[float]

#     # 获取当前道路速度
#     expected_time = float(0)
#     avg_speeds = []

#     for edge_id in path:
#         edge = netData.getEdge(edge_id)
#         # 检查道路长度是否大于50
#         edge_length = netData.getEdge(edge_id).length
#         if edge_length > 100:
#             avg_speed = traci.edge.getLastStepMeanSpeed(edge)
#         else:
#             avg_speed = traci.edge.getMaxSpeed(edge_id)

#         # 确保速度不为0
#         if avg_speed == 0:
#             avg_speed = traci.edge.getMaxSpeed(edge_id)

#         speed = round(avg_speed, 2)
#         avg_speeds.append(speed)

#     # 计算路径通行时间
#     for i in range(len(path)):
#         expected_time += path_lengths[i] / avg_speeds[i]

#     return expected_time


def add_detectors_on_path(netData, path):
    """
    在给定路径的每条车道上动态添加检测器，位置设为车道长度的90%。
    
    :param netData: SUMO网络数据访问对象
    :param path: 路径，由边的ID组成的列表
    """
    for edge_id in path:
        edge = netData.getEdge(edge_id)
        for lane in edge.getLanes():
            lane_id = lane.getID()
            lane_length = lane.getLength()
            detector_position = min(lane_length * 0.9, lane_length - 0.01)  # 确保不超过车道长度，且略微小于100%以避免边界问题
            
            # 使用TraCI命令添加检测器，这里以inductionLoop（感应环）为例
            detector_id = "detector_{}_{}".format(edge_id, lane_id.split("_")[-1])
            traci.inductionloop.add(detector_id, lane_id, detector_position, duration=3600)  # 假设持续时间为1小时，按需调整
            
            # 注意：实际使用时，需要确保traci已初始化并处于模拟运行状态
            # print(f"Detector {detector_id} added at position {detector_position} on lane {lane_id}")
            

# light_count, light_ids = count_and_return_traffic_lights(node_ids, net_path)

# print(f"路口ID {node_ids} 存在红绿灯组的个数为 {light_count}")
# print(f"路口ID {node_ids} 的红绿灯ID按顺序为 {light_ids}")

# def get_traffic_light_ids(net_path, edge_ids):

# from __future__ import print_function
# #此处应该再添加一步，获得路口的红绿灯ID后，放置检测器，然后每到一个路口就返回检测结果
# #******
# def get_expected_passing_times(edge_ids, net_path, simulation_step=None):
#     #计算路径中每个路段的预计通过时间。

#     edge_lengths = get_edge_lengths(edge_ids, net_path)
#     expected_times = []

#     # Load the net.xml file using sumolib
#     net = sumolib.net.readNet(net_path)

#     # 通过TraCI接口获取每个路段的平均车速
#     if simulation_step is not None:
#         traci.init(sumoCmd=["sumo-gui", "-c", "your_sumo_config.sumocfg"])  # 初始化TraCI与SUMO
#         traci.simulationStep(simulation_step)  # 设置仿真到指定步数
        
#         for edge_id in edge_ids:
#             edge = net.getEdge(edge_id)
#             # 假设能直接通过某种方式（这在真实应用中需要更复杂的逻辑）获取平均速度
#             avg_speed = traci.edge.getLastStepMeanSpeed(edge_id)  # 假设存在这样的函数直接获取平均车速
            
#             if avg_speed > 0:  # 防止除以零错误
#                 expected_time = edge_lengths[edge_ids.index(edge_id)] / avg_speed
#                 expected_times.append(expected_time)
#             else:
#                 # 如果车速为0，可能需要特殊处理，比如设置一个默认值或忽略
#                 expected_times.append(float('inf'))  # 或者其他合适的处理方式
                
#         traci.close()  # 关闭TraCI连接
#     else:
#         print("未提供仿真步数，无法动态获取车速。请提供仿真步数或预先设定车速。")
#         # 如果没有仿真步数，这里需要其他方式获取车速，否则无法继续

    # return expected_times
# ********************************************************
# 与GPT4进行动态交互
# ********************************************************

import json
import requests
api_url= "http"
api_key = "160ecb3"

def create_chat_completion(api_key, messages, temperature=0.5):
    headers = {
        "Content-Type": "application/json",
        "Authorization": f"Bearer {api_key}"
    }
    data = {
        "model": "gpt-3.5-turbo",  # 或者其他你想要使用的模型
        "messages": messages,
        "temperature": temperature
    }
    
    response = requests.post(
        api_url,
        headers=headers,
        data=json.dumps(data)
    )
    
    if response.status_code == 200:
        return response.json()["choices"][0]["message"]["content"]
    else:
        response.raise_for_status()
# K_shortest_paths = [['a','b','c','d'],['a','e','f','d'],['a','g','h','d']]
# total_time = [60,50,10]
# traffic_light_count = [2,2,2]
# error_edge = ['g']


# print(best_path)  # 输出：['a', 'g', 'h', 'd']
# print(choice_reason)

# 示例使用
# k_shortest_paths = [['edge1', 'edge2', 'edge3'], ['edge4', 'edge5', 'edge6'], ['edge7', 'edge8']]
# evaluated_paths = [(['edge1', 'edge2', 'edge3'], 120.0), (['edge4', 'edge5', 'edge6'], 130.0), (['edge7', 'edge8'], 110.0)]
# best_path = evaluate_and_choose_path(k_shortest_paths, evaluated_paths)
# print(f"The best path chosen by GPT is: {best_path}")
def calculate_total_time(shortest_paths): 
    total_time = [] 
    for path in shortest_paths:
        time = 0
        for edge in path:
            vehicle_count = traci.edge.getLastStepVehicleNumber(edge)
            if vehicle_count == 0:
                passing_speed = traci.edge.getMaxSpeed(edge)
            else:
                vehicle_speed = traci.edge.getLastStepVehicleNumber(edge)
                passing_speed = vehicle_speed / vehicle_count
            edge_length = traci.edge.getLength(edge)
            pass_time = edge_length / passing_speed
            time += pass_time
        total_time.append(time)
    return total_time





#**************************************************************************************************
# 小车仿真运行
#**********************************************************************************************

# 确保在调用run()前已经加载或初始化了netData
# netData = sumolib.net.readNet(net_file)

def run():
    sumoBinary = sumo_path + "sumo-gui" #启动sumo仿真
    sumoCmd = [sumoBinary, "-n", net_file, "-r", rou_file,"-c",cfg_file] #构建一个命令行参数列表,用于启动SUMO仿真
    traci.start(sumoCmd)#使用traci库启动仿真
    step = 0
    pass_edges = []#存储车辆通过的边信息
    pass_node = []#存储车辆通过的点信息    
    # 获取最短路径并初始化车辆
    v_id = "v1"
    route = get_shortest_path(start_edge, end_edge)#此处得到的route即为list[str]的形式
    route_id = "theorshortest_path"
    traci.route.add(route_id, route)
    traci.vehicle.add(v_id,route_id)
    while traci.simulation.getMinExpectedNumber() > 0:#仿真循环  获取当前仿真步骤中预期的最小车辆数，若该值大于0,则继续仿真
        traci.simulationStep()#通过调用函数执行下一步仿真
        step += 1#step记录当前仿真步骤的编号，每次执行仿真步骤后，sep+1
        sim_time = traci.simulation.getTime()#获取当前仿真时间=sim_time
        
        if v_id in traci.vehicle.getIDList():
            traci.gui.setZoom('View #0',8017.95)
            traci.gui.trackVehicle('View #0',v_id)
            edge_ = traci.vehicle.getLaneID(v_id).split("_")[0]
            if str(edge_).startswith(":"):
                pass
            else:
                edge_node = netData.getEdge(edge_).getToNode()
                #获取所在路口
                nod_id = edge_node.getID()
                node_type = edge_node.getType()
            #将一条边的ID添加到pass_edges列表中，如果该边已经存在于列表中，则跳过该边
            if edge_ in pass_edges:
                pass
            else:
                pass_edges.append(edge_)
            #pass_edges是list[str]
            if node_type == "traffic_light":

                # 获取信号灯信息
                phase = traci.trafficlight.getPhase(nod_id)
                # 剩余时长
                duration = traci.trafficlight.getNextSwitch(nod_id) - sim_time
                print(f"当前车辆所在车道：{edge_}, 节点名称：{nod_id}, 节点属性：{node_type}")
                print(f"路口信号灯当前相位: {phase}, 剩余时长： {duration}")
                    
                    # 记录经过的信号灯
                if  nod_id in pass_node:
                    #获取当前边在list[str]中的site
                    #保证所选取的边是按我预期执行    
                    position = edge_ids.index(edge_)
                    try:
                        target_edge= edge_ids[position+2]
                    except IndexError:
                        target_edge= end_edge
                    total_time = []
                    traffic_light_count = []
                    k_shortest_paths = find_k_shortest_paths(G, edge_, target_edge, 3)
                    for path in k_shortest_paths:
                        path_nodes= get_node_id(edge_,path)
                        num_lights = count_traffic_lights(path_nodes)
                        traffic_light_count.append(num_lights)
                        # total_time.append(get_travel_time(path))
                    total_time=calculate_path_travel_time(k_shortest_paths)
                    for i, path in enumerate(k_shortest_paths, 1):
                        travel_time = calculate_path_travel_time(G, path)
                        print(f"路径 {i}: {path}")
                    # shortest_path = []
                    # chose_path =[]
                    #此处应添加一个解析路径得node_ids的函数
                    traffic_light_count = count_traffic_lights()
                    prompt=f'''
                    你是一位正在城市道路上驾驶汽车的司机，
                    现在你行驶到了一个路口,
                    接下来你希望选择哪条路线，有以下至多三条路线list[path1,path2,path3]
                    可供选择{k_shortest_paths},
                    其中每条路线上的红绿灯数量为{traffic_light_count},
                    通过这三条路径的理论时间为{total_time},[pass_path1_time,pass_path2_time,pass_path3_time],total_time的单位为s
                    请输出选择的用时最短路径，并输出选择理由，例如
                    假设
                    K_shortest_paths = [['edge1','edge2','edge3','edge7'],['edge1','edge4','edge7'],['edge1','edge5','edge6','edge7','edge8','edge7']]
                    total_time = [30,40,50]
                    traffic_light_count = [3,2,4]
                    error_edge = ['edge2']
                    输出：
                    best_path = ['edge1','edge4','edge7']
                    choice_reason = '从total——time来看，path1<path2<path3,
                                    从traffic_light_count来看，path2<path1<path3,
                                    从error_edge来看，不能选择包括'edge2'的路径， 即不能选择path1
                                    综合考虑，所以选择path2'
                    '''
                    messages = [{"role": "user", "content": prompt}]
                    response_text = create_chat_completion(api_key, messages)
                    def parse_message(message):
                        # 切割message字符串，获取best_path和choice_reason
                        best_path = message.split("best_path = ")[1].split("\n")[0]
                        choice_reason = message.split("choice_reason = ")[1].strip().strip("''")

                        # 处理best_path字符串，去除首尾空格和引号，然后按逗号分割成列表
                        best_path = best_path.strip("[]'").split("', '")

                        return best_path, choice_reason
                    best_path, choice_reason = parse_message(response_text )
                    chose_path, choice_reason = parse_message(response_text )
                    first_element_index =shortest_path.index[chose_path[0]]
                    last_element_index = shortest_path.index[chose_path[-1]]
                    best_path= shortest_path[:first_element_index]+chose_path+shortest_path[last_element_index+1:]
                    print(f"当前路径动态更改为 {best_path}")
                    # 更新车辆路线
                    traci.vehicle.changeRoute(v_id, "new_route", best_path)
                    print(f"Vehicle {v_id} changed route to {best_path}")
                    # # 打印结果和通行时间
                    if i == 1:
                            # traci.route.add(str(sim_time), path)
                            # traci.vehicle.add("vid_{}".format(sim_time), str(sim_time))
                                # 修改车道形状颜色->模拟路径
                        for edge_id in path:
                            edge_shape = traci.lane.getShape(edge_id + "_0")
                            polygon_id = "highlight_{}_{}".format(edge_id, sim_time)
                            traci.polygon.add(
                                polygonID=polygon_id,
                                shape=edge_shape,
                                color=(255, 0, 0, 255),
                                fill=True,
                                layer=0
                                )

                        print(f"路径 {i}:  行程时间: {travel_time:.2f} seconds")
                else:
                    pass_node.append(nod_id)
                
    traci.close()


import random
random.seed(4)

# 程序入口
if __name__ == '__main__':
    sumo_path = os.path.join(os.environ['SUMO_HOME'], 'bin') + "/"
    net_file = net_path5  # 路网文件
    rou_file = rou_path5  # 路网文件
    cfg_file = sumocfg_path5
    netData = sumolib.net.readNet(net_file)  # 读取路网文件
   # 随机选择初始化路径
    # edges = netData.getEdges()
    # edge_ids = [edge.getID() for edge in edges]
    # start_edge = random.choice(edge_ids) 
    # end_edge = random.choice(edge_ids)
    start_edge = "-30482615#4"
    end_edge = "-315358244"
    edges = netData.getEdges()
    # shortest_path, edge_ids,node_ids = get_shortest_path(start_edge, end_edge)
    # print(f"理论最短路径: {shortest_path}")
    # 加载网络
    # Grah = save_network()
    G = load_network()
    # 开始运行仿真
    run()

