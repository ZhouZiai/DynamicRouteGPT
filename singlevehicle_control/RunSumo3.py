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

def get_node_id(start_edge_id,shortest_path):
    start_edge = netData.getEdge(start_edge_id)    
    # 从最短路径中提取路口ID
    node_ids = [start_edge.getFromNode().getID()]
    for edge in shortest_path:
        node_ids.append(edge.getToNode().getID())
    
    return node_ids
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

###################################################################
# ********************************************************
# 与GPT4进行动态交互
# ********************************************************

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

def run():
    sumoBinary = sumo_path + "sumo-gui"  # -gui就是打开gui界面
    sumoCmd = [sumoBinary, "-n", net_file, "-r", rou_file,"-c", cfg_file]
    traci.start(sumoCmd)

    # # # 设置仿真时间偏移
    # start_time_offset = 57600.00  # 从路网文件中获取的起始时间
    # traci.simulationStep(start_time_offset)  # 将仿真时间设置为起始时间
    # delay =100
    step = 0
    # 经过的路段
    pass_edges = []
    # 经过的路口
    pass_node = []
    # 两点之间的路径
    # route = traci.simulation.findRoute(start_edge, end_edge)
    # v_id = "v1"
    # if route is not None:
    #     route_id = "random_route"
    #     traci.route.add(route_id, route.edges)
    #     traci.vehicle.add(v_id, route_id)
    shortest_path, shortest_edge_list= get_shortest_path(start_edge, end_edge)
    #测试代码,测试通过
    print(f"理论最短路径：{shortest_edge_list}")
    v_id = "v1"
    if shortest_path is not None:
        route_id = "shortest_route"
        traci.route.add(route_id, shortest_edge_list)
        traci.vehicle.add(v_id, route_id)
    # simulationEnd = shortest_path[-1]###
    # while traci.simulation.getArrivedIDList() != simulationEnd:####
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        step += 1
        sim_time = traci.simulation.getTime()
    ####################################################################### 以上代码为车辆添加行驶路线
        if v_id in traci.vehicle.getIDList():
            traci.gui.setZoom('View #0', 1000)
                #设置视角跟随车辆v1
            traci.gui.trackVehicle('View #0', v_id)
            #     # 获取车辆所处车道
            edge_ = traci.vehicle.getLaneID(v_id).split("_")[0]
                #测试效果
            print(f"当前车辆所在车道edge_：{edge_}")
            # traci.vehicle.rerouteEffort(v_id)
            # traci.vehicle.setRoute(v_id, shortest_edge_list)
            # vehicle_count = traci.edge.getLastStepVehicleNumber(edge_)
            # if vehicle_count > 0:
            #     print(f"当前车辆所在车道：{edge_}, 车辆数量：{vehicle_count}")
            # 获取所在的路口
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
                if node_type == "traffic_light":
                
                    #获取信号灯信息
                    # phase = traci.trafficlight.getPhase(nod_id)
                    #剩余时长
                    duration = traci.trafficlight.getNextSwitch(nod_id) - sim_time
                    print(f"当前车辆所在车道：{edge_}, 节点名称nod_id：{nod_id}, 节点属性node_type：{node_type}")
                #     # print(f"路口信号灯当前相位: {phase}, 剩余时长： {duration}")
                #     # 记录经过的信号灯
                    if nod_id in pass_node:
                        ################################################测试代码（pass）
                        # if edge_ in shortest_edge_list:
                        if duration ==0:
                            traci.simulation.pause()
                            position =shortest_edge_list.index(edge_)
                            try:
                                target_edge = shortest_edge_list[position+2]
                            except:
                                target_edge = end_edge
                            print(f"node_id is{nod_id}，target_edge is{target_edge}")
                            k_shortest_paths = find_k_shortest_paths(G, edge_, target_edge, 3)
                                # # 打印结果和通行时间
                            for i, path in enumerate(k_shortest_paths, 1):
                                print(f"备选路径有以下几条 {i}: {path}")
                            estimated_time_list = get_estimated_time(k_shortest_paths)
                            print(f"总通行时间：{estimated_time_list}")
                                    ###########继续测试代码
                            shoretest_time = min(estimated_time_list)
                            chose_path_id = estimated_time_list.index(shoretest_time)
                            chose_path = k_shortest_paths[chose_path_id]
                            print(f'chose_path is {chose_path}')
                            print(type(chose_path))
                                    # 更新车辆路线
                            first_element_index =shortest_edge_list.index(chose_path[0])
                            last_element_index = shortest_edge_list.index(chose_path[-1])
                            best_path= shortest_edge_list[:first_element_index]+chose_path+shortest_edge_list[last_element_index+1:]
                            traci.route.add(str(sim_time), best_path)
                            traci.vehicle.add("vid_{}".format(sim_time), str(sim_time)) 
                            traci.simulation.resume()
                            #for path in k_shortest_paths:
                            #     path_nodes= get_node_id(edge_,path)
                            #     num_lights = count_traffic_lights(path_nodes)
                            #     traffic_light_count.append(num_lights)
                            # traffic_light_count = count_traffic_lights() 
                            # prompt=f'''
                            # 你是一位正在城市道路上驾驶汽车的司机，
                            # 现在你行驶到了一个路口,
                            # 接下来你希望选择哪条路线，有以下至多三条路线list[path1,path2,path3]
                            # 可供选择{k_shortest_paths},
                            # 其中每条路线上的红绿灯数量为{traffic_light_count},
                            # 通过这三条路径的理论时间为{total_time},[pass_path1_time,pass_path2_time,pass_path3_time],total_time的单位为s
                            # 请输出选择的用时最短路径，并输出选择理由，例如
                            # 假设
                            # K_shortest_paths = [['edge1','edge2','edge3','edge7'],['edge1','edge4','edge7'],['edge1','edge5','edge6','edge7','edge8','edge7']]
                            # total_time = [30,40,50]
                            # traffic_light_count = [3,2,4]
                            # error_edge = ['edge2']
                            # 输出：
                            # best_path = ['edge1','edge4','edge7']
                            # choice_reason = '从total——time来看，path1<path2<path3,
                            #                 从traffic_light_count来看，path2<path1<path3,
                            #                 从error_edge来看，不能选择包括'edge2'的路径， 即不能选择path1
                            #                 综合考虑，所以选择path2'
                            # '''
                            # messages = [{"role": "user", "content": prompt}]
                            # response_text = create_chat_completion(api_key, messages) 
                            # def parse_message(message):
                            # # 切割message字符串，获取best_path和choice_reason
                            # best_path = message.split("best_path = ")[1].split("\n")[0]
                            # choice_reason = message.split("choice_reason = ")[1].strip().strip("''")

                            # # 处理best_path字符串，去除首尾空格和引号，然后按逗号分割成列表
                            # best_path = best_path.strip("[]'").split("', '")

                            # return best_path, choice_reason                     
                            # best_path, choice_reason = parse_message(response_text )
                            # chose_path, choice_reason = parse_message(response_text )
                            # first_element_index =shortest_path.index[chose_path[0]]
                            # last_element_index = shortest_path.index[chose_path[-1]]
                            # best_path= shortest_path[:first_element_index]+chose_path+shortest_path[last_element_index+1:]
                            # print(f"当前路径动态更改为 {best_path}")
                            #  # 修改车道形状颜色->模拟路径
                            # for edge_id in path:
                            #     edge_shape = traci.lane.getShape(edge_id + "_0")
                            #     polygon_id = "highlight_{}_{}".format(edge_id, sim_time)
                            #     traci.polygon.add(
                            #         polygonID=polygon_id,
                            #         shape=edge_shape,
                            #         color=(255, 0, 0, 255),
                            #         fill=True,
                            #         layer=0
                            #     )
                        else:
                            continue
                else:
                    pass_node.append(nod_id)
        # #             #     print(f"本次循环中车辆行驶到了{edge_}边")


    traci.close()


# 程序入口
# if __name__ == '__main__':
#     sumo_path = os.path.join(os.environ['SUMO_HOME'], 'bin') + "/"
#     net_file = r'D:\intereting\DynamicNavigation\findPath\findPath\ingolstadt21.net.xml'  # 路网文件
#     rou_file = r'D:\intereting\DynamicNavigation\findPath\findPath\ingolstadt21.rou.xml'  # 路网文件
#     cfg_file = r'D:\intereting\DynamicNavigation\findPath\findPath\ingolstadt21.sumocfg'
#     netData = sumolib.net.readNet(net_file)  # 读取路网文件
#     start_edge = "-30482615#4"
#     end_edge = "-315358244"
#     edges = netData.getEdges()
#     # 加载网络
#     # Grah = save_network()
#     G = load_network()
#     # 开始运行仿真
#     run()



###########58043   |||(3)58228

###现在是的问题是，决策时间会对决策行为产生影响，进而对决策行为产生影响，所以，如果能减少决策数量，可能结果会好点
if __name__ == '__main__':
    sumo_path = os.path.join(os.environ['SUMO_HOME'], 'bin') + "/"
    net_file = r'D:\intereting\DynamicNavigation\findPath\findPath\test_data\Xrouting\test.net.xml'  # 路网文件
    rou_file = r'D:\intereting\DynamicNavigation\findPath\findPath\test_data\Xrouting\test.rou.xml'  # 路网文件
    cfg_file = r'D:\intereting\DynamicNavigation\findPath\findPath\test_data\Xrouting\test.sumocfg'
    netData = sumolib.net.readNet(net_file)  # 读取路网文件
    start_edge = "right0D0"
    end_edge = "A2left2"
    edges = netData.getEdges()
    # 加载网络
    Grah = save_network()
    G = load_network()
    # 开始运行仿真
    run()