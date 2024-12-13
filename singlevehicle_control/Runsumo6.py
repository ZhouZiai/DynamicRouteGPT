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
from sumolib import checkBinary


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

##################################################################
#测试通过的代码
#################################################################
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


def get_edge_length(edge_id):  
    #获取路径中涉及到的路段的长度
    # edge_lengths = []
    # edge_length =0
    # Iterate over the given edge IDs and retrieve their lengths
    # for edge_id in shortest_node_list:
    edge = netData.getEdge(edge_id)
    edge_length = edge.getLength()
        # edge_lengths.append(length)

    return edge_length

def edge_Maxspeed(edge_id):
    # Iterate over the given edge IDs and retrieve their speed limits
    edge = netData.getEdge(edge_id)
    speed_limit = edge.getSpeed()
    return speed_limit

def get_estimated_time(k_shortest_paths):
    total_time = []
    try:
        for path in k_shortest_paths:
            estimated_time = 0
            for edge in path:
                vehicle_count = traci.edge.getLastStepVehicleNumber(edge)
                
                if vehicle_count > 0:
                    vehicle_ids_on_edge = traci.edge.getLastStepVehicleIDs(edge)
                    total_speed = sum(traci.vehicle.getSpeed(i) for i in vehicle_ids_on_edge)
                    average_speed = total_speed / vehicle_count
                    average_speed = round(average_speed, 2)
                else:
                    average_speed = edge_Maxspeed(edge)  # 使用之前计算的结果，减少冗余调用
                    
                edge_length = get_edge_length(edge)
                
                # 避免除以零
                if average_speed == 0:
                    average_speed = edge_Maxspeed(edge)
                
                edge_pass_time = edge_length / average_speed
                edge_pass_time = round(edge_pass_time, 2)
                estimated_time += edge_pass_time
                
            estimated_time = round(estimated_time, 2)  # 最终结果四舍五入
            total_time.append(estimated_time)
    
    except Exception as e:
        print(f"An error occurred: {e}")
        # 根据您的需要，这里可以添加更多的错误处理逻辑，例如回退到上一个状态或重试连接。
    
    return total_time        

# 调用函数

def is_vehicle_at_edge_end(vehicle_id, edge_id, edge_length):
    """
    检查车辆是否在边的末端。
    """
    position = traci.vehicle.getPosition(vehicle_id)
    return position == (edge_id, edge_length)

def change_vehicle_target_if_needed(vehicle_id, chosen_path):
    """
    如果车辆到达当前边的末端，则更改其目标为下一条边。
    """
    if chosen_path:
        current_edge = traci.vehicle.getLaneID(vehicle_id).split("_")[0]
        if is_vehicle_at_edge_end(vehicle_id, current_edge, get_edge_length(current_edge)):
            traci.vehicle.changeTarget(vehicle_id, chosen_path.pop(0))

def simulate(chose_path,v_id):
    simulationEnd = chose_path[-1]
    while traci.simulation.getArrivedIDList() != simulationEnd:  # 确保比较操作的正确性
        traci.simulationStep()
        for sub_edge in chose_path:
            if traci.vehicle.getRoadID(v_id) == sub_edge:
                change_vehicle_target_if_needed(v_id, chose_path)
                break  # 车辆目标变更后，无需继续遍历剩余路径
        change_vehicle_target_if_needed(v_id, chose_path)  # 检查是否在循环外到达了边的末端
        # 输出车辆状态或其他处理
        print(f"Vehicle {v_id} now on edge {traci.vehicle.getRoadID(v_id)}")

# 注意：假设v_id和chose_path已经在函数外部正确定义和初始化             

###定义绕行函数，
def avoidEdge(vehId, edgeId):
    """Sets an edge's travel time for a vehicle infinitely high, and reroutes the vehicle based on travel time.
    Args:
        vehId (Str): The ID of the vehicle to reroute.
        edgeId (Str): The ID of the edge to avoid.
    """
    traci.vehicle.setAdaptedTraveltime(
        vehId, edgeId, float('inf'))
    traci.vehicle.rerouteTraveltime(vehId)

#####定义更改车辆颜色
def setVehColor(vehId, color):
    """Changes a vehicle's color.
    Args:
        vehId (String): The vehicle to color.
        color ([Int, Int, Int]): The RGB color to apply.
    """
    traci.vehicle.setColor(vehId, color)

import xml.etree.ElementTree as ET

def get_node_id(start_edge_id,path):
    start_edge = netData.getEdge(start_edge_id)    
    # 从最短路径中提取路口ID
    node_ids = [start_edge.getFromNode().getID()]
    for edge in path:
        node_ids.append(edge.getToNode().getID())
    
    return node_ids
def count_traffic_lights(node_ids, net_path):
    #计算路径中的红绿灯个数以及返回节点ID,此处无法省略
    count = 0
    # light_ids = []

    # Load the net.xml file
    tree = ET.parse(net_path)
    root = tree.getroot()

    # Search for tlLogic elements with matching IDs
    for tl_logic in root.findall(".//tlLogic"):
        if tl_logic.attrib['id'] in node_ids:
            count += 1
            # light_ids.append(tl_logic.attrib['id'])

    return count#, light_ids


import json
import requests
api_url= "https://gpt-api.hkust-gz.edu.cn/v1/chat/completions"
api_key = "160ecb30bc154b9dac0d0209bfae62955a5577f813cc45b9a21cb97aa1cb01d6"

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
def parse_message(message):
    # 切割message字符串，获取best_path和choice_reason
    chose_path = message.split("chose_path = ")[1].split("\n")[0]
    choice_reason = message.split("choice_reason = ")[1].strip().strip("''")
    # 处理best_path字符串，去除首尾空格和引号，然后按逗号分割成列表
    chose_path = chose_path.strip("[]'").split("', '")
    return chose_path, choice_reason

import time
def run():
    sumoBinary = sumo_path + "sumo-gui"  # -gui就是打开gui界面
    sumoCmd = [sumoBinary, "-n", net_file, "-r", rou_file,"-c", cfg_file]
    traci.start(sumoCmd)
    step = 0
    # 经过的路段
    pass_edges = []
    # 经过的路口
    pass_node = []
    total_CO2 =0
    total_fuel =0
    v_id = "v0"
# 获取当前仿真中所有车辆的ID列表
    vehicle_ids = traci.vehicle.getIDList()
    print(vehicle_ids)
    # 检查特定车辆ID是否在列表中，从而判断车辆是否存在
    if v_id in vehicle_ids:
        # 获取车辆当前的路线ID
        route_id = traci.vehicle.getRouteID(v_id)
        print(route_id)
        # 获取路线的边缘列表
        route_edges = traci.route.getEdges(route_id)
        print(route_edges)
        # 起点即路线中的第一个边，终点为最后一个边
        start_edge = route_edges[0]
        end_edge = route_edges[-1]
        
        print(f"Vehicle {v_id} is on route {route_id}.")
        print(f"Start edge: {start_edge}, End edge: {end_edge}")
    else:
        print(f"Vehicle {v_id} does not exist in the simulation.")
    shortest_path, shortest_edge_list= get_shortest_path(start_edge, end_edge)
    #测试代码,测试通过
    print(f"理论最短路径：{shortest_edge_list}")
    # vehicle_ids =["v1"]
    if shortest_path is not None:
        route_id = "shortest_route"
        traci.route.add(route_id, shortest_edge_list)
        traci.vehicle.add(v_id, route_id)

    while True:
        traci.simulationStep()
        # current_vehicles = set(traci.vehicle.getIDList())
        simulation_start_time = traci.simulation.getTime() 
        if not v_id :
            print("All vehicles have left the network.")
            elapsed_time = traci.simulation.getTime() - simulation_start_time
            print(f"Total time from the first vehicle's departure to the last one leaving: {elapsed_time:.2f} seconds")
            break
        step += 1
        sim_time = traci.simulation.getTime()
        #以上代码为车辆添加行驶路线
        if v_id in traci.vehicle.getIDList():
            traci.gui.setZoom('View #0', 1000)
            #设置视角跟随车辆v1
            traci.gui.trackVehicle('View #0', "v1")
            # 获取车辆所处车道
            edge_ = traci.vehicle.getLaneID(v_id).split("_")[0]
            #测试效果
            print(f"当前车辆所在车道edge_：{edge_}")
            CO2=traci.vehicle.getCOEmission("v1")
            total_CO2 +=CO2
            print(f"当前车辆CO2的总排放量：{total_CO2:2f}")
            fuel = traci.vehicle.getFuelConsumption("v1")
            total_fuel +=fuel
            print(f"当前车辆总油耗：{total_fuel:2f}")
            if str(edge_).startswith(":"):
                pass
            else:
                edge_node = netData.getEdge(edge_).getToNode()
                # 获取路口属性
                nod_id = edge_node.getID()
                print(f"到达路口{nod_id}")
                node_type = edge_node.getType()
                print(f"当前路口{nod_id}的属性是{node_type}")
                # 记录经过的路段
                if edge_ in pass_edges:
                    pass
                else:
                    pass_edges.append(edge_)
                    if nod_id in pass_node:
                        traci.simulation.pause()
                        position =shortest_edge_list.index(edge_)
                        try:
                            target_edge = shortest_edge_list[position+2]
                        except:
                            target_edge = end_edge
                        print(f"node_id is{nod_id}，target_edge is{target_edge}")
                        k_shortest_paths = find_k_shortest_paths(G, edge_, target_edge, 3)
                        # 打印结果和通行时间
                        for i, path in enumerate(k_shortest_paths, 1):
                            print(f"备选路径有以下几条 {i}: {path}")
                        estimated_time_list = get_estimated_time(k_shortest_paths)
                        traffic_light_count =[]
                        for path in k_shortest_paths:
                            node_ids = get_node_id(edge_,path)
                            light_count = count_traffic_lights(node_ids,net_file)
                            traffic_light_count.append(light_count)
                        prompt=f'''
                            你是一位正在城市道路上驾驶汽车的司机，
                            现在你行驶到了一个路口,
                            接下来你希望选择哪条路线，有以下至多三条路线list[path1,path2,path3]
                            可供选择{k_shortest_paths},
                            其中每条路线上的红绿灯数量为{traffic_light_count},
                            通过这三条路径的理论时间为{estimated_time_list},[pass_path1_time,pass_path2_time,pass_path3_time],total_time的单位为s
                            请输出选择的用时最短路径，并输出选择理由，例如
                            假设
                            K_shortest_paths = [['edge1','edge2','edge3','edge7'],['edge1','edge4','edge7'],['edge1','edge5','edge6','edge7','edge8','edge7']]
                            total_time = [30,40,50]
                            traffic_light_count = [3,2,4]
                            error_edge = ['edge2']
                            输出：
                            chose_path = ['edge1','edge4','edge7']
                            choice_reason = '从total——time来看，path1<path2<path3,
                                            从traffic_light_count来看，path2<path1<path3,
                                            从error_edge来看，不能选择包括'edge2'的路径， 即不能选择path1
                                            综合考虑，所以选择path2'
                            '''
                        messages = [{"role": "user", "content": prompt}]
                        response_text = create_chat_completion(api_key, messages)
                        chose_path, choice_reason = parse_message(response_text )
                        first_element_index =shortest_edge_list.index(chose_path[0])
                        last_element_index = shortest_edge_list.index(chose_path[-1])
                        print(f'chose_path is {chose_path},chose_reason is {choice_reason}')
                        # 更新车辆路线
                        best_path= shortest_edge_list[:first_element_index]+chose_path+shortest_edge_list[last_element_index+1:]
                        traci.route.add(str(sim_time), best_path)
                        traci.vehicle.add("vid_{}".format(sim_time), str(sim_time)) 
                        traci.simulation.resume()
                    else:
                        pass_node.append(nod_id)
    traci.close()


# if __name__ == '__main__':
#     sumo_path = os.path.join(os.environ['SUMO_HOME'], 'bin') + "/"
#     net_file = r'D:\\intereting\\DynamicNavigation\\findPath\\findPath\\test_data\\Xrouting\\test.net.xml'  # 路网文件
#     rou_file = r'D:\\intereting\\DynamicNavigation\\findPath\\findPath\\test_data\\Xrouting\\test.rou.xml'  # 路网文件
#     cfg_file = r'D:\\intereting\\DynamicNavigation\\findPath\\findPath\\test_data\\Xrouting\\test.sumocfg'
#     netData = sumolib.net.readNet(net_file)  # 读取路网文件
#     start_edge = "right0D0"
#     end_edge = "A2left2"
#     edges = netData.getEdges()
#     # 加载网络
#     # Grah = save_network()
#     G = load_network()
#     # 开始运行仿真
#     run()
if __name__ == '__main__':
    sumo_path = os.path.join(os.environ['SUMO_HOME'], 'bin') + "/"
    # net_file = r'D:\\intereting\\DynamicNavigation\\findPath\\findPath\\test_data\\孙倩\\grid4x4\\grid4x4.net.xml'  # 路网文件
    # rou_file = r'D:\\intereting\\DynamicNavigation\\findPath\\findPath\\test_data\\孙倩\\grid4x4\\grid4x4.rou.xml'  # 路网文件
    # cfg_file = r'D:\\intereting\\DynamicNavigation\\findPath\\findPath\\test_data\\孙倩\\grid4x4\\grid4x4.sumocfg'
    # netData = sumolib.net.readNet(net_file)
    # start_edge = "right1D1"
    # end_edge = "A0left0"
    net_file = r'D:\\intereting\\DynamicNavigation\\findPath\\findPath\\test_data\\孙倩\\Hangzhou-4x4\\hangzhou.net.xml'  # 路网文件
    rou_file = r'D:\\intereting\\DynamicNavigation\\findPath\\findPath\\test_data\\孙倩\\Hangzhou-4x4\\Hangzhou_5.rou.xml'  # 路网文件
    cfg_file = r'D:\\intereting\\DynamicNavigation\\findPath\\findPath\\test_data\\孙倩\\Hangzhou-4x4\\hangzhou.sumocfg'
    netData = sumolib.net.readNet(net_file) 
    edges = netData.getEdges()
    # 加载网络
    # Grah = save_network()
    G = load_network()
    # 开始运行仿真
    run()