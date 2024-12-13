import os, sys

if 'SUMO_HOME' in os.environ:  # 检查SUMO_HOME是否在系统环境变量中，进行下一步仿真的必要条件
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')

    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

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


def load_network():
    # 从文件加载图
    with open("network_route.pkl", 'rb') as f:
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
    route = traci.simulation.findRoute(start_edge, end_edge)
    v_id = "v1"
    if route is not None:
        route_id = "random_route"
        traci.route.add(route_id, route.edges)
        traci.vehicle.add(v_id, route_id)
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        step += 1
        sim_time = traci.simulation.getTime()

        if v_id in traci.vehicle.getIDList():
            # traci.gui.setZoom('View #0', 8017.95)
            # # 设置视角跟随车辆v1
            # traci.gui.trackVehicle('View #0', v_id)
            # 获取车辆所处车道
            edge_ = traci.vehicle.getLaneID(v_id).split("_")[0]
            # 获取所在的路口
            if str(edge_).startswith(":"):
                pass
            else:
                edge_node = netData.getEdge(edge_).getToNode()
                # 获取路口属性
                nod_id = edge_node.getID()
                node_type = edge_node.getType()
                # 记录经过的路段
                if edge_ in pass_edges:
                    pass
                else:
                    pass_edges.append(edge_)
                if node_type == "traffic_light":

                    # 获取信号灯信息
                    phase = traci.trafficlight.getPhase(nod_id)
                    # 剩余时长
                    duration = traci.trafficlight.getNextSwitch(nod_id) - sim_time
                    print(f"当前车辆所在车道：{edge_}, 节点名称：{nod_id}, 节点属性：{node_type}")
                    print(f"路口信号灯当前相位: {phase}, 剩余时长： {duration}")

                    # 记录经过的信号灯
                    if nod_id in pass_node:
                        # 获取前三条最短路径
                        k_shortest_paths = find_k_shortest_paths(G, edge_, end_edge, 3)
                        # # 打印结果和通行时间
                        for i, path in enumerate(k_shortest_paths, 1):
                            travel_time = calculate_path_travel_time(G, path)
                            print(f"路径 {i}: {path}")

                            if i == 1:
                                traci.route.add(str(sim_time), path)
                                traci.vehicle.add("vid_{}".format(sim_time), str(sim_time))
                                # # 修改车道形状颜色->模拟路径
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

                            print(f"路径 {i}:  行程时间: {travel_time:.2f} seconds")
                    else:
                        pass_node.append(nod_id)

    traci.close()


# 程序入口
if __name__ == '__main__':
    sumo_path = os.path.join(os.environ['SUMO_HOME'], 'bin') + "/"
    net_file = r'D:\intereting\DynamicNavigation\findPath\findPath\test_data\Xrouting\test.net.xml'  # 路网文件
    rou_file = r'D:\intereting\DynamicNavigation\findPath\findPath\test_data\Xrouting\test.rou.xml'  # 路网文件
    cfg_file = r'D:\intereting\DynamicNavigation\findPath\findPath\test_data\Xrouting\test.rou.xml'
    netData = sumolib.net.readNet(net_file)  # 读取路网文件
    start_edge = "right0D0"
    end_edge = "A2left2"
    edges = netData.getEdges()
    # 加载网络
    Grah = save_network()
    G = load_network()
    # 开始运行仿真
    run()
