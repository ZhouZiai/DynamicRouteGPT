import os, sys

if 'SUMO_HOME'in os.environ:  # 检查SUMO_HOME是否在系统环境变量中，进行下一步仿真的必要条件
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')

    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import traci
import sumolib
import networkx as nx
import pickle
from itertools import islice
from sumolib import checkBinary

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
    with open("grid_route5.pkl", 'wb') as f:
        pickle.dump(G, f)
    return G

graph_path = r'D:\intereting\DynamicNavigation\findPath\grid_route5.pkl'
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
    total_time = 0
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
def edge_Maxspeed(edge_id):
    # Iterate over the given edge IDs and retrieve their speed limits
    edge = netData.getEdge(edge_id)
    speed_limit = edge.getSpeed()
    return speed_limit
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
def get_node_id(start_edge_id,path):
    start_edge = netData.getEdge(start_edge_id)    
    # 从最短路径中提取路口ID
    node_ids = [start_edge.getFromNode().getID()]
    for edge_id in path:
        edge = netData.getEdge(edge_id)
        if edge:
            node_ids.append(edge.getToNode().getID())
        else:
            print(f"警告：边ID {edge_id} 对应的边不存在于网络中。")
    return node_ids
import xml.etree.ElementTree as ET
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

# 定义解析result的函数
def parse_result(result):
    parts = result.split("chose_path is ")
    if len(parts) > 1:
        chose_path_str = parts[1].split("],")[0] + "]"  # 解析路径
        chose_path_str = chose_path_str.replace("'", "")  # 移除单引号
        chose_path = eval(chose_path_str)  # 使用eval将字符串转为列表
        choice_reason = parts[1].split("],", 1)[1].strip()  # 解析理由
        return chose_path, choice_reason
    else:
        return None, None


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


def run():
    sumoBinary = sumo_path + "sumo-gui"  # -gui就是打开gui界面
    sumoCmd = [sumoBinary, "-n", net_file, "-r", rou_file,"-c", cfg_file]
    traci.start(sumoCmd)
    step = 0
    # 经过的路段
    pass_edges = []
    # 经过的路口
    pass_node = []
    vehicle_ids = []
    all_shortest_paths =[]
    for i in range(car_num):
        vehicle_id = f"veh_{i}"
        route_id = f"route_{i}"
        start_edge = start_edges[i]
        end_edge = end_edges[i]
        # 计算最短路径，注意这里假设get_shortest_path返回两个值，但只使用第二个作为实际路径
        _, shortest_edge_list = get_shortest_path(start_edge, end_edge)
        # 添加路线到路网中
        traci.route.add(route_id, shortest_edge_list)       
        # 创建车辆并分配路线
        traci.vehicle.add(vehicle_id, route_id,depart= depart_times[i],departPos="210",departSpeed=10)
        # 收集车辆ID
        vehicle_ids.append(vehicle_id)
        all_shortest_paths.append(shortest_edge_list)
    step = 0
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        step += 1
        sim_time = traci.simulation.getTime()
        # edge_list = [f"edge_{i}" for i in range(1, car_num+1)]
        # if v_id in traci.vehicle.getIDList():
        #     traci.gui.setZoom('View #0', 1000)#8017.95
        #     #设置视角跟随车辆v1
        #     traci.gui.trackVehicle('View #0', v_id)
            # 获取车辆所处车道
            # edge_ = traci.vehicle.getLaneID(v_id).split("_")[0]
        for vechicle_id in vehicle_ids:
            sate =  vehicle_ids.index(vechicle_id)
            # new_road = f"road_{sate}"
            if vechicle_id in traci.vehicle.getIDList():
                # traci.simulation.pause()
                edge_ = traci.vehicle.getLaneID(vechicle_id).split("_")[0]
                if str(edge_).startswith(":"):
                    pass
                else:
                    edge_node = netData.getEdge(edge_).getToNode()
                    # 获取路口属性
                    nod_id = edge_node.getID()
                    print(f"{vechicle_id}到达路口{nod_id}")
                    node_type = edge_node.getType()
                    print(f"当前路口{nod_id}的属性是{node_type}")
                    # 记录经过的路段
                    if edge_ in pass_edges:
                        pass
                    else:
                        pass_edges.append(edge_)
                        if nod_id in pass_node:
                            # traci.simulation.pause()
                            short_edge_list = all_shortest_paths[sate]
                            position =short_edge_list.index(edge_)
                            try:
                                target_edge = short_edge_list[position+2]
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
                            traci.simulationStep(0) 
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
                            # chose_path, choice_reason = parse_result(response_text)
                            print(f'chose_path is {chose_path}\n,choice_reason is {choice_reason}')
                            first_element_index =short_edge_list.index(chose_path[0])
                            last_element_index = short_edge_list.index(chose_path[-1])
                            try:
                                last_element_index = short_edge_list.index(chose_path[-1])
                            except ValueError:
                                print(f"'{chose_path[-1]}'未在 short_edge_list 中找到，请检查数据或逻辑。short_edge_list is {short_edge_list }")
                            # 更新车辆路线
                            best_route= shortest_edge_list[:first_element_index]+chose_path+short_edge_list[last_element_index+1:]
                            print(f"best_route is {best_route}")
                            # 假设vid_to_update是需要更新路线的车辆ID，已知车辆当前在"A"，新路径为best_route=["A","D","B","C"]
                            traci.route.add(str(sim_time), best_route)
                            # 假设我们已经有了新的最优路径best_path，并且需要更新车辆"vid_to_update"
                            vid_to_update = "{}".format(vechicle_id)
                            # 确保车辆已经存在并且在仿真中活动
                            if traci.vehicle.getRoadID(vid_to_update):
                                # 获取车辆当前的边（这里简化处理，实际可能需要更复杂的逻辑来确保车辆确实位于"A"）
                                current_edge = traci.vehicle.getRoadID(vid_to_update)
                                # 确保车辆当前位于新路径的起点
                                if current_edge == best_route[0]:
                                    # 从当前仿真中移除车辆，以便能够更改其路线
                                    traci.vehicle.remove(vid_to_update)
                                    # 添加新的路线到仿真中
                                    # unique_route_id = f"route_{sim_time}"
                                    traci.route.add(str(sim_time), best_route)  # 假设sim_time用于生成唯一的路线ID
                                    traci.vehicle.addFull(vid_to_update, best_route, departLane='random', departSpeed='max', arrivalLane='current', arrivalSpeed='current')
                                else:
                                    print(f"Vehicle {vid_to_update} is not at the expected starting edge of the new route.")
                            else:
                                print(f"Vehicle {vid_to_update} not found in the simulation.")

                            traci.simulationStep(1)
                        else:
                            pass_node.append(nod_id)
            
    traci.close()

# 程序入口
if __name__ == '__main__':
    sumo_path = os.path.join(os.environ['SUMO_HOME'], 'bin') + "/"
    net_file = r'D:\\intereting\\DynamicNavigation\\findPath\\findPath\\test_data\\grid4x4\\grid4x4.net.xml'# 路网文件
    rou_file = r'D:\\intereting\\DynamicNavigation\\findPath\\findPath\\test_data\\grid4x4\\grid4x4.rou.xml'# 路网文件
    cfg_file = r'D:\\intereting\\DynamicNavigation\\findPath\\findPath\\test_data\\grid4x4\\grid4x4.sumocfg'
    netData = sumolib.net.readNet(net_file)
    print(netData)
    car_num  =19
    depart_times= [820, 1055, 1128, 1603, 1662, 1694, 1712, 1713, 1770, 1800, 1801, 1873, 1877, 1977, 2004, 2045, 2126, 2305, 2746]
    start_edges= ['top2C3', 'top2C3', 'top2C3', 'top2C3', 'top2C3', 'top2C3', 'top2C3', 'top2C3', 'top2C3', 'top2C3', 'top2C3', 'top2C3', 'top2C3', 'top2C3', 'top2C3', 'top2C3', 'top2C3', 'top2C3', 'top2C3']
    end_edges= ['C3B3', 'C3B3', 'C3B3', 'C3B3', 'C3B3', 'C3B3', 'C3B3', 'C3B3', 'C3B3', 'D0right0', 'C3B3', 'C3B3', 'C3B3', 'C3B3', 'C3B3', 'C3B3', 'C3B3', 'C3B3', 'C3B3']
    edges = netData.getEdges()
    G = load_network()
    run()
