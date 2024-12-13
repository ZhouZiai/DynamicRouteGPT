import xml.etree.ElementTree as ET

def process_xml(xml_file, output_file):
    # 解析XML文件
    tree = ET.parse(xml_file)
    root = tree.getroot()
    
    # 初始化计数器和累加器
    tripinfo_count = 0
    total_arrival = 0.0
    total_duration = 0.0
    total_waiting_time = 0.0
    total_time_loss = 0.0
    
    # 遍历所有的<tripinfo>元素
    for tripinfo in root.findall('tripinfo'):
        tripinfo_count += 1
        total_arrival += float(tripinfo.get('arrival'))
        total_duration += float(tripinfo.get('duration'))
        total_waiting_time += float(tripinfo.get('waitingTime'))
        total_time_loss += float(tripinfo.get('timeLoss'))
    
    # 写入结果到文本文件
    with open(output_file, 'w') as f:
        f.write(f'tripinfo id的个数：{tripinfo_count}\n')
        f.write(f'所有 arrival值的总和：{total_arrival:.2f}\n')
        f.write(f'所有duration值的总和：{total_duration:.2f}\n')
        f.write(f'所有waitingTime 值的总和：{total_waiting_time:.2f}\n')
        f.write(f'所有timeLoss值的总和：{total_time_loss:.2f}\n')

# 调用函数处理文件
process_xml(r'D:\test_dataset\test\test_p15_result.xml', r'D:\test_dataset\test\test_p15_result.txt')