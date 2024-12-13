import os,sys

if 'SUMO_HOME'in os.environ:  # 检查SUMO_HOME是否在系统环境变量中，进行下一步仿真的必要条件
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
    with open("network_grid.pkl",'wb') as f:
        pickle.dump(G,f)
    return G

# graph_path = r'D:\intereting\DynamicNavigation\findPath\findPath\network_route.pkl'
#gridpath
graph_path = r'D:\intereting\DynamicNavigation\findPath\findPath\test_data\孙倩\grid4x4\network_grid.pkl'
#Hangzhou
# graph_path = r'D:\intereting\DynamicNavigation\findPath\findPath\test_data\孙倩\Hangzhou-4x4\hangzhou_route.pkl'

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
        for vechicle_id in vehicle_ids:
            sate = vehicle_ids.index(vechicle_id)
            if vechicle_id in traci.vehicle.getIDList():
                edge_ = traci.vehicle.getLaneID(vechicle_id).split("_")[0]
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
                            k_shortest_paths = find_k_shortest_paths(G,edge_,end_edge,3)
                                # # 打印结果和通行时间
                            for i,path in enumerate(k_shortest_paths,1):
                                travel_time = calculate_path_travel_time(G,path)
                                print(f"当前路口{nod_id}路径 {i}: {path}")
                                if i == 1:
                                    traci.route.add(str(sim_time),path)
                                print(f"路径 {i}:  行程时间: {travel_time:.2f} seconds")
                            # traci.simulation.resume()
                        else:
                            pass_node.append(nod_id)

    traci.close()

# for grid
if __name__ == '__main__':
    sumo_path = os.path.join(os.environ['SUMO_HOME'],'bin') + "/"
    cfg_file=r'D:\intereting\DynamicNavigation\findPath\findPath\test_data\孙倩\grid4x4\grid4x4.sumocfg'
    net_file= r'D:\intereting\DynamicNavigation\findPath\findPath\test_data\孙倩\grid4x4\grid4x4.net.xml'
    rou_file=r'D:\intereting\DynamicNavigation\findPath\findPath\test_data\孙倩\grid4x4\grid4x4.rou.xml'
    netData = sumolib.net.readNet(net_file)  # 读取路网文件
    car_num  =13
    depart_times=[1017,1322,1386,1613,1653,1787,1803,1844,1851,1920,1971,2047,2471]
    start_edges=['left0A0','left0A0','left0A0','left0A0','left0A0','top1B3','top1B3','top1B3','top1B3','top1B3','top1B3','top1B3','top1B3']
    end_edges=['right0D0','right0D0','right0D0','right0D0','right0D0','right0D0','right0D0','right0D0','right0D0','right0D0','right0D0','right0D0','right0D0']
    edges = netData.getEdges()
    G = load_network()
    run()

# depart_times=[1017,1322,1386,1613,1653,1787,1803,1844,1851,1920,1971,2047,2471]
# start_edges=['left0A0','left0A0','left0A0','left0A0','left0A0','top1B3','top1B3','top1B3','top1B3','top1B3','top1B3','top1B3','top1B3']
# end_edges=['right0D0','right0D0','right0D0','right0D0','right0D0','right0D0','right0D0','right0D0','right0D0','right0D0','right0D0','right0D0','right0D0']

# depart_times=[443, 2049]
# start_edges=['left0A0', 'left0A0']
# end_edges=['right0D0', 'B3A3']
# depart_times=[112,1395,1417,1495,1501,1605,1662,1684,1810,1994]
# depart_times=['top2C3','top2C3','top2C3','left0A0','left0A0','left0A0','left0A0','left0A0','left0A0','left0A0']
# end_edges=['B3A3','B3A3','B3A3','B3A3','B3A3','B3A3','B3A3','B3A3','B3A3','B3A3']
# depart_times=[ 287,488,547,604,695,1449,1508,1665,1681,1730,1789,1815,1834,2010,2162.00]
# start_edges=[top2C3','top2C3','top2C3','top2C3','top2C3','top2C3','top2C3','top2C3','top2C3','top2C3_0,top2C3','top2C3','top2C3','top2C3','top2C3_0]
# end_edges=[left0A0','left0A0','left0A0','left0A0','left0A0','left0A0','left0A0','left0A0','left0A0','left0A0_1,left0A0','left0A0','left0A0','left0A0','left0A0_0]

# depart_times=[1800, 2045, 2126, 2306]
# start_edges=[top2C3,top2C3,top2C3,top2C3]
# end_edges=[D0right0,C3B3,C3B3,C3B3]
# depart_times=[863,926,1096,1395,1397,1819,1872,1874,1929,1931,1996,1998,2031,2033,2038,2159,2161,2746,2970,3171.00]
# start_edges=['bottom1B0','bottom1B0','bottom1B0','bottom1B0','bottom1B0','bottom1B0','bottom1B0','bottom1B0','bottom1B0','bottom1B0','bottom1B0','bottom1B0','bottom1B0','bottom1B0','bottom1B0','bottom1B0','bottom1B0','bottom1B0','bottom1B0','bottom1B0']
# end_edges=['B0C0','B0C0','B0C0','B0C0','B0C0','B0C0','B0C0','B0C0','B0C0','B0C0_0,B0C0','B0C0','B0C0','B0C0','B0C0','B0C0','B0C0','B0C0','B0C0','B0C0_0']
# depart_times=[1393,1612,1796,1805,1814,1871,1927,2173,2179,2333,2612,3325]
# arrivel=['left2A2','left2A2','left2A2','left2A2','left2A2','left2A2','left2A2','left2A2','left2A2','left2A2','left2A2','left2A2']
# # end_edges=['A2A1','A2A1','A2A1','A2A1','A2A1','A2A1','A2A1','A2A1','A2A1','A2A1','A2A1','A2A1']
# depart_times=[820,1055,1128,1603,1603,1662,1694,1712,1714,1770,1801,1877,1977,2004]
# start_edges=[top2C3,top2C3,top2C3,top2C3,top2C3,top2C3,top2C3,top2C3,top2C3,top2C3,top2C3,top2C3,top2C3,top2C3]
# end_edges=[C3B3,C3B3,C3B3,C3B3_2, C3B3_2, C3B3,C3B3,C3B3,C3B3,C3B3,C3B3,C3B3,C3B3,C3B3]
