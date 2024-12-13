import xml.etree.ElementTree as ET

def extract_tripinfo(input_file, output_file, id_prefix):
    # 解析XML文件
    tree = ET.parse(input_file)
    root = tree.getroot()

    # 创建一个新的ElementTree实例用于输出文件
    new_root = ET.Element("tripinfos")
    new_tree = ET.ElementTree(new_root)

    # 提取包含特定id前缀的<tripinfo>元素
    for tripinfo in root.findall('tripinfo'):
        if tripinfo.get('id').startswith(id_prefix):
            new_root.append(tripinfo)

    # 写入新的XML文件
    new_tree.write(output_file, encoding="UTF-8", xml_declaration=True)

# 使用函数
input_xml_file = r'D:\test_dataset\test\test_p15.xml'
output_xml_file = r'D:\test_dataset\test\test_p15_result.xml'
id_prefix = 'veh_'

extract_tripinfo(input_xml_file, output_xml_file, id_prefix)