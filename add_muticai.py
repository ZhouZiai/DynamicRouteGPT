import xml.etree.ElementTree as ET
from xml.dom import minidom
import numpy as np
np.random.seed(0)  # 设置随机数生成器的种子
grid_num = 1494
hangzhou_num = 1660
existing_grid_routes = [i for i in range(grid_num)] 
existing_hangzhou_routes = [i for i in range(hangzhou_num)] # 创建包含1494个整数的列表
selected_grid_route = np.random.choice(existing_grid_routes) 
# print(f"selected_grid_route is {selected_grid_route}") 
# print("route is depart= 1747.00 right1D1 D1C1 C1C0 C0B0 B0A0 A0left0")
existing_hangzhou_routes = [i for i in range(hangzhou_num)]  # 创建包含1494个整数的列表
selected_hangzhou_route = np.random.choice(existing_hangzhou_routes) 
# print(f"selected_hangzhou_route is {selected_hangzhou_route}")# 从existing_routes中随机选择一个元素
# print("route is depart=636, road_1_5_3 road_1_4_3 road_1_3_3 road_1_2_2")
grid_5=['4', '31', '34', '52', '54', '82', '89', '141', '142', '148', '152', '191', '192', '231', '254', '260', '278', '302', '322', '326', '333', '347', '361', '362', '364', '376', '408', '420', '427', '434', '446', '458', '483', '487', '529', '535', '554', '558', '651', '674', '758', '761', '838', '853', '857', '863', '884', '897', '900', '930', '942', '961', '1001', '1030', '1031', '1054', '1061', '1069', '1081', '1095', '1114', '1122', '1131', '1157', '1186', '1278', '1283', '1336', '1350', '1400', '1411', '1442', '1463']
grid_10=['22', '33', '56', '60', '71', '92', '104', '117', '118', '129', '138', '146', '180', '181', '182', '183', '195', '213', '216', '217', '224', '227', '243', '246', '248', '276', '302', '306', '311', '329', '333', '336', '337', '352', '378', '380', '388', '396', '399', '406', '410', '412', '418', '423', '436', '454', '465', '471', '495', '496', '499', '501', '509', '533', '537', '542', '543', '545', '547', '548', '556', '571', '577', '592', '599', '600', '611', '615', '630', '641', '643', '648', '661', '668', '672', '673', '678', '681', '692', '695', '701', '719', '742', '760', '762', '783', '786', '787', '798', '800', '802', '806', '816', '831', '840', '841', '868', '872', '885', '898', '905', '906', '918', '933', '954', '961', '965', '968', '982', '998', '999', '1005', '1009', '1038', '1062', '1063', '1074', '1083', '1086', '1102', '1112', '1114', '1147', '1149', '1156', '1166', '1179', '1224', '1250', '1255', '1257', '1261', '1267', '1293', '1294', '1307', '1318', '1320', '1321', '1328', '1340', '1362', '1373', '1385', '1416', '1448', '1466'] 
grid_20=['0', '6', '18', '20', '29', '42', '43', '45', '48', '55', '56', '59', '63', '72', '75', '84', '90', '95', '96', '108', '116', '117', '119', '121', '125', '127', '130', '134', '142', '143', '148', '151', '152', '154', '158', '159', '162', '163', '165', '168', '170', '179', '181', '186', '187', '192', '194', '195', '210', '212', '214', '218', '220', '223', '237', '246', '254', '265', '267', '268', '269', '272', '275', '277', '278', '282', '291', '294', '301', '302', '304', '308', '313', '320', '328', '330', '334', '337', '339', '343', '359', '371', '379', '383', '385', '392', '394', '397', '401', '402', '410', '411', '412', '416', '420', '430', '437', '447', '453', '456', '468', '474', '481', '487', '497', '500', '514', '524', '525', '528', '530', '532', '533', '540', '543', '545', '547', '548', '556', '559', '567', '571', '584', '585', '586', '592', '595', '599', '600', '606', '608', '610', '627', '630', '634', '636', '637', '639', '640', '646', '650', '652', '660', '666', '667', '668', '678', '679', '684', '704', '707', '708', '712', '715', '716', '717', '719', '720', '728', '730', '734', '736', '737', '739', '745', '747', '761', '771', '780', '790', '792', '793', '799', '800', '806', '828', '842', '844', '851', '854', '856', '861', '867', '870', '872', '873', '878', '895', '896', '903', '904', '915', '924', '933', '948', '965', '968', '969', '972', '977', '978', '980', '982', '991', '995', '1002', '1005', '1017', '1029', '1030', '1040', '1042', '1044', '1051', '1055', '1057', '1066', '1067', '1072', '1073', '1075', '1077', '1082', '1083', '1094', '1096', '1098', '1109', '1115', '1124', '1125', '1126', '1132', '1133', '1138', '1139', '1144', '1154', '1156', '1157', '1160', '1163', '1177', '1188', '1192', '1201', '1220', '1225', '1227', '1246', '1250', '1254', '1256', '1257', '1259', '1263', '1264', '1265', '1267', '1274', '1275', '1279', '1284', '1286', '1305', '1307', '1315', '1319', '1320', '1321', '1328', '1329', '1332', '1338', '1343', '1360', '1375', '1379', '1382', '1398', '1406', '1410', '1415', '1419', '1420', '1422', '1435', '1440', '1459', '1461', '1463', '1464', '1465', '1466', '1471']
grid_30=['6', '14', '15', '16', '20', '22', '23', '25', '29', '39', '40', '42', '50', '52', '55', '56', '58', '63', '70', '72', '80', '89', '98', '101', '105', '107', '108', '111', '112', '115', '119', '120', '122', '123', '127', '130', '133', '134', '136', '142', '143', '144', '146', '151', '160', '164', '165', '166', '170', '173', '178', '179', '181', '186', '187', '191', '193', '195', '197', '201', '202', '205', '206', '214', '219', '227', '232', '233', '234', '247', '252', '254', '255', '256', '257', '260', '261', '262', '263', '270', '272', '273', '279', '283', '286', '287', '290', '302', '304', '311', '312', '316', '318', '328', '329', '332', '334', '335', '337', '339', '340', '343', '345', '346', '358', '359', '364', '369', '371', '374', '375', '379', '386', '394', '401', '405', '406', '407', '409', '413', '423', '425', '427', '429', '432', '433', '438', '443', '447', '451', '458', '462', '464', '467', '468', '471', '472', '473', '478', '479', '482', '485', '489', '499', '502', '506', '510', '512', '514', '516', '519', '526', '527', '528', '532', '533', '535', '536', '539', '542', '547', '551', '554', '555', '557', '559', '563', '569', '572', '581', '583', '592', '595', '596', '605', '609', '614', '617', '619', '620', '625', '626', '627', '628', '629', '634', '650', '654', '660', '661', '664', '668', '675', '676', '677', '679', '683', '684', '685', '687', '691', '692', '693', '694', '696', '698', '699', '705', '706', '708', '712', '713', '715', '719', '720', '722', '734', '735', '737', '738', '740', '742', '747', '759', '764', '766', '767', '771', '772', '773', '775', '781', '782', '785', '786', '787', '792', '797', '798', '800', '808', '809', '816', '828', '830', '831', '834', '836', '838', '839', '840', '844', '853', '856', '858', '861', '867', '868', '869', '871', '872', '873', '875', '878', '882', '885', '887', '888', '893', '894', '897', '910', '912', '913', '916', '919', '930', '934', '944', '946', '948', '949', '952', '957', '958', '959', '961', '962', '969', '971', '973', '977', '979', '980', '981', '982', '984', '988', '992', '993', '994', '995', '997', '1006', '1007', '1009', '1010', '1016', '1022', '1027', '1036', '1037', '1040', '1051', '1052', '1053', '1056', '1061', '1068', '1069', '1079', '1080', '1081', '1090', '1093', '1094', '1097', '1098', '1101', '1102', '1103', '1108', '1111', '1117', '1118', '1124', '1129', '1133', '1135', '1137', '1138', '1140', '1143', '1147', '1154', '1156', '1157', '1158', '1159', '1161', '1170', '1172', '1174', '1181', '1182', '1183', '1186', '1187', '1188', '1194', '1196', '1201', '1202', '1209', '1211', '1213', '1214', '1215', '1216', '1218', '1223', '1228', '1229', '1231', '1238', '1240', '1241', '1242', '1250', '1259', '1262', '1263', '1272', '1276', '1277', '1278', '1280', '1289', '1291', '1294', '1295', '1303', '1316', '1320', '1321', '1322', '1323', '1328', '1337', '1342', '1343', '1344', '1345', '1346', '1349', '1350', '1353', '1360', '1362', '1364', '1365', '1366', '1371', '1373', '1379', '1380', '1384', '1385', '1387', '1396', '1398', '1399', '1400', '1402', '1403', '1404', '1405', '1407', '1410', '1416', '1418', '1428', '1429', '1432', '1442', '1444', '1445', '1451', '1454', '1460', '1466', '1468']
grid_40=['1', '4', '16', '18', '20', '22', '23', '27', '29', '32', '33', '36', '37', '38', '39', '42', '43', '44', '45', '46', '49', '50', '52', '55', '56', '58', '59', '60', '63', '64', '70', '71', '72', '73', '75', '77', '80', '83', '90', '94', '96', '98', '102', '105', '106', '109', '110', '111', '112', '116', '118', '119', '125', '127', '134', '135', '140', '144', '147', '150', '151', '152', '154', '156', '157', '158', '161', '162', '166', '167', '169', '174', '176', '177', '178', '181', '183', '187', '189', '190', '196', '199', '200', '201', '202', '205', '208', '210', '212', '214', '215', '217', '218', '221', '224', '226', '229', '233', '234', '238', '244', '249', '251', '254', '257', '259', '260', '261', '265', '269', '273', '274', '276', '277', '278', '282', '294', '295', '296', '297', '303', '304', '306', '311', '313', '315', '317', '323', '330', '334', '338', '342', '344', '347', '349', '350', '352', '358', '367', '368', '369', '370', '373', '374', '376', '379', '380', '382', '383', '384', '386', '387', '388', '389', '391', '392', '394', '396', '399', '400', '403', '404', '406', '409', '410', '421', '423', '427', '428', '429', '431', '434', '438', '440', '441', '442', '444', '448', '449', '450', '451', '452', '454', '455', '456', '458', '459', '460', '461', '462', '463', '466', '467', '469', '473', '480', '481', '482', '485', '487', '491', '495', '498', '501', '506', '507', '508', '509', '511', '514', '516', '519', '521', '523', '524', '526', '527', '528', '533', '536', '537', '540', '541', '543', '546', '548', '553', '558', '562', '563', '565', '566', '567', '569', '572', '573', '574', '576', '582', '583', '586', '588', '590', '591', '593', '595', '600', '601', '604', '611', '612', '613', '616', '617', '625', '626', '628', '629', '630', '631', '632', '633', '637', '638', '640', '641', '642', '645', '646', '647', '651', '653', '656', '659', '669', '674', '677', '679', '682', '683', '684', '686', '687', '691', '692', '693', '694', '696', '700', '701', '703', '705', '706', '707', '711', '712', '713', '714', '717', '719', '720', '721', '722', '725', '726', '727', '728', '734', '736', '737', '740', '748', '752', '755', '757', '758', '769', '771', '772', '775', '776', '779', '787', '790', '791', '793', '796', '797', '798', '800', '802', '803', '804', '805', '808', '811', '814', '815', '817', '819', '821', '822', '823', '825', '827', '830', '833', '834', '837', '838', '839', '842', '844', '845', '846', '848', '852', '857', '858', '860', '863', '865', '866', '867', '869', '877', '878', '880', '881', '886', '887', '888', '889', '891', '902', '909', '912', '914', '915', '916', '917', '923', '924', '927', '929', '930', '932', '933', '935', '937', '939', '940', '942', '944', '945', '947', '953', '955', '958', '961', '962', '963', '971', '972', '977', '978', '979', '982', '984', '985', '987', '992', '994', '995', '997', '1003', '1012', '1017', '1018', '1019', '1021', '1022', '1024', '1030', '1031', '1032', '1033', '1034', '1036', '1039', '1040', '1042', '1045', '1052', '1054', '1055', '1057', '1061', '1062', '1063', '1064', '1065', '1067', '1068', '1070', '1072', '1074', '1077', '1081', '1085', '1087', '1090', '1092', '1101', '1102', '1104', '1106', '1115', '1116', '1118', '1119', '1121', '1124', '1125', '1128', '1130', '1132', '1133', '1135', '1138', '1143', '1148', '1151', '1154', '1155', '1156', '1157', '1159', '1161', '1165', '1166', '1167', '1168', '1171', '1176', '1177', '1178', '1179', '1181', '1184', '1188', '1189', '1191', '1192', '1197', '1199', '1208', '1212', '1213', '1216', '1217', '1218', '1219', '1220', '1222', '1224', '1226', '1230', '1234', '1235', '1236', '1237', '1241', '1245', '1246', '1254', '1260', '1263', '1264', '1275', '1276', '1280', '1282', '1286', '1289', '1292', '1293', '1294', '1297', '1299', '1301', '1302', '1303', '1304', '1314', '1316', '1317', '1319', '1321', '1323', '1324', '1325', '1326', '1327', '1332', '1336', '1338', '1339', '1340', '1343', '1348', '1350', '1351', '1354', '1356', '1359', '1360', '1363', '1364', '1369', '1373', '1378', '1381', '1385', '1387', '1389', '1394', '1395', '1398', '1401', '1404', '1406', '1407', '1415', '1425', '1426', '1428', '1433', '1435', '1443', '1447', '1451', '1458', '1459', '1461', '1466', '1467', '1469', '1470']
grid_50=['0', '1', '4', '6', '7', '8', '9', '11', '12', '17', '18', '19', '23', '27', '30', '31', '32', '33', '36', '37', '38', '41', '42', '43', '45', '46', '50', '51', '53', '55', '58', '59', '60', '61', '65', '66', '67', '70', '71', '76', '83', '85', '91', '93', '95', '96', '98', '100', '104', '105', '106', '107', '111', '114', '116', '119', '121', '122', '126', '127', '128', '131', '134', '136', '137', '138', '139', '140', '146', '149', '150', '153', '154', '158', '164', '168', '174', '176', '177', '178', '179', '180', '183', '184', '185', '187', '188', '190', '191', '192', '193', '195', '196', '197', '198', '199', '200', '201', '204', '206', '208', '210', '211', '220', '221', '224', '227', '228', '232', '233', '234', '240', '242', '245', '247', '249', '250', '251', '253', '254', '255', '257', '259', '260', '261', '263', '264', '267', '270', '271', '272', '273', '275', '277', '278', '280', '281', '285', '287', '289', '292', '295', '296', '298', '300', '302', '303', '304', '305', '306', '308', '309', '310', '314', '315', '317', '320', '324', '325', '327', '328', '329', '332', '334', '336', '337', '339', '342', '343', '347', '348', '354', '358', '361', '362', '363', '365', '368', '369', '372', '374', '376', '379', '381', '384', '385', '386', '387', '389', '390', '393', '394', '396', '397', '398', '403', '404', '405', '406', '408', '410', '412', '413', '416', '417', '419', '421', '424', '425', '426', '427', '428', '429', '431', '436', '441', '442', '445', '447', '449', '451', '454', '455', '458', '462', '468', '471', '474', '477', '478', '479', '485', '486', '487', '488', '489', '495', '504', '505', '508', '510', '512', '513', '518', '519', '520', '521', '524', '526', '528', '530', '532', '533', '536', '537', '538', '541', '542', '544', '547', '549', '552', '558', '560', '561', '563', '567', '569', '573', '574', '575', '577', '579', '580', '581', '582', '584', '585', '587', '589', '591', '592', '593', '594', '595', '596', '597', '598', '601', '603', '604', '607', '608', '612', '615', '618', '619', '620', '622', '623', '625', '629', '632', '634', '635', '636', '638', '639', '646', '648', '650', '652', '654', '656', '658', '659', '663', '664', '666', '669', '671', '674', '676', '677', '678', '679', '681', '689', '690', '691', '692', '694', '701', '702', '703', '705', '706', '708', '712', '713', '717', '718', '719', '720', '723', '724', '726', '729', '732', '734', '737', '744', '745', '746', '747', '749', '754', '757', '758', '760', '762', '763', '764', '765', '768', '770', '771', '774', '775', '776', '777', '779', '780', '781', '785', '787', '788', '789', '790', '793', '797', '799', '801', '802', '804', '807', '810', '812', '814', '815', '817', '818', '820', '821', '825', '826', '827', '831', '832', '833', '834', '838', '839', '841', '843', '846', '847', '848', '850', '851', '853', '854', '855', '857', '859', '860', '863', '866', '869', '870', '871', '873', '875', '877', '878', '879', '880', '883', '884', '887', '888', '892', '894', '899', '903', '905', '906', '907', '909', '910', '917', '920', '921', '923', '925', '927', '928', '933', '935', '937', '938', '939', '940', '944', '945', '946', '947', '950', '951', '952', '956', '957', '959', '960', '961', '964', '966', '968', '969', '970', '972', '975', '977', '979', '980', '982', '983', '985', '992', '994', '995', '996', '997', '998', '1001', '1003', '1004', '1005', '1006', '1007', '1008', '1009', '1010', '1013', '1016', '1018', '1025', '1026', '1027', '1028', '1030', '1031', '1033', '1034', '1035', '1036', '1041', '1042', '1045', '1046', '1048', '1049', '1051', '1053', '1054', '1055', '1057', '1058', '1059', '1061', '1065', '1067', '1068', '1070', '1075', '1076', '1077', '1079', '1083', '1084', '1085', '1086', '1087', '1089', '1091', '1092', '1095', '1096', '1097', '1099', '1101', '1102', '1104', '1106', '1109', '1110', '1112', '1113', '1115', '1116', '1118', '1120', '1121', '1124', '1126', '1127', '1129', '1130', '1131', '1137', '1140', '1141', '1143', '1148', '1149', '1150', '1151', '1154', '1157', '1159', '1160', '1162', '1164', '1165', '1166', '1167', '1169', '1171', '1172', '1173', '1175', '1176', '1178', '1181', '1183', '1184', '1185', '1188', '1191', '1192', '1194', '1195', '1196', '1197', '1198', '1199', '1200', '1204', '1205', '1208', '1210', '1215', '1219', '1220', '1222', '1223', '1224', '1227', '1229', '1230', '1231', '1232', '1233', '1234', '1236', '1237', '1239', '1240', '1244', '1245', '1251', '1252', '1253', '1255', '1256', '1257', '1258', '1259', '1261', '1268', '1269', '1270', '1273', '1274', '1276', '1278', '1279', '1280', '1281', '1282', '1283', '1285', '1286', '1288', '1289', '1290', '1293', '1301', '1305', '1306', '1307', '1314', '1316', '1317', '1321', '1322', '1323', '1325', '1326', '1327', '1328', '1330', '1332', '1333', '1335', '1336', '1338', '1340', '1346', '1349', '1350', '1355', '1356', '1357', '1359', '1361', '1364', '1365', '1367', '1368', '1369', '1370', '1374', '1377', '1379', '1380', '1383', '1384', '1385', '1386', '1388', '1389', '1393', '1394', '1396', '1398', '1400', '1401', '1403', '1404', '1405', '1406', '1412', '1413', '1414', '1415', '1416', '1418', '1419', '1421', '1422', '1423', '1425', '1426', '1430', '1431', '1434', '1437', '1439', '1440', '1441', '1447', '1448', '1449', '1450', '1451', '1452', '1454', '1455', '1457', '1458', '1461', '1465', '1468', '1470', '1471']

Hangzhou_5=['0', '16', '52', '59', '71', '75', '99', '100', '129', '132', '137', '144', '145', '173', '176', '188', '252', '285', '304', '319', '338', '352', '394', '426', '433', '475', '477', '481', '492', '497', '518', '534', '539', '550', '553', '595', '608', '641', '651', '656', '679', '695', '702', '718', '724', '757', '766', '773', '778', '781', '829', '844', '851', '863', '901', '935', '953', '989', '1003', '1008', '1016', '1027', '1051', '1060', '1066', '1080', '1088', '1138', '1174', '1211', '1263', '1280', '1309', '1310', '1311', '1390', '1391', '1411', '1447', '1450', '1472', '1487', '1494', '1560', '1566', '1570', '1578', '1633']
Hangzhou_10=['1', '11', '36', '44', '47', '70', '71', '86', '95', '97', '102', '108', '112', '136', '145', '154', '177', '183', '199', '200', '211', '219', '239', '244', '264', '269', '284', '288', '308', '335', '342', '343', '371', '400', '401', '405', '418', '468', '482', '488', '491', '495', '503', '512', '520', '538', '550', '557', '562', '566', '579', '584', '592', '599', '601', '602', '610', '619', '624', '625', '634', '643', '644', '661', '663', '664', '681', '708', '710', '721', '722', '729', '735', '743', '745', '748', '757', '763', '772', '784', '801', '805', '807', '818', '821', '854', '855', '856', '858', '864', '868', '919', '931', '939', '946', '958', '963', '966', '984', '992', '1009', '1041', '1049', '1050', '1052', '1054', '1070', '1076', '1087', '1105', '1110', '1127', '1137', '1141', '1160', '1167', '1168', '1175', '1178', '1194', '1205', '1220', '1228', '1230', '1243', '1245', '1250', '1261', '1265', '1281', '1293', '1307', '1318', '1337', '1350', '1353', '1362', '1393', '1396', '1405', '1406', '1407', '1413', '1420', '1421', '1445', '1468', '1473', '1499', '1504', '1517', '1520', '1522', '1531', '1538', '1540', '1566', '1570', '1581', '1583', '1612', '1623', '1626', '1630', '1634', '1643']
Hangzhou_20=['8', '14', '21', '22', '25', '28', '33', '34', '45', '47', '48', '49', '51', '56', '76', '78', '80', '85', '88', '97', '99', '105', '112', '131', '143', '147', '148', '154', '159', '160', '163', '166', '168', '174', '175', '182', '192', '195', '196', '197', '201', '207', '212', '216', '217', '219', '221', '226', '233', '235', '236', '259', '260', '261', '264', '266', '267', '282', '289', '299', '301', '310', '314', '317', '321', '325', '328', '329', '331', '336', '340', '342', '351', '352', '354', '357', '364', '365', '372', '377', '382', '388', '393', '394', '397', '406', '409', '410', '420', '422', '424', '430', '431', '434', '436', '439', '440', '441', '449', '450', '452', '456', '461', '463', '467', '468', '469', '470', '481', '487', '488', '494', '499', '500', '508', '517', '519', '521', '523', '525', '528', '529', '538', '542', '544', '549', '559', '561', '564', '570', '575', '583', '585', '594', '596', '618', '623', '628', '644', '646', '650', '658', '678', '694', '697', '698', '700', '701', '704', '711', '714', '717', '723', '724', '731', '732', '734', '736', '741', '748', '754', '758', '759', '761', '775', '778', '780', '783', '793', '794', '803', '806', '809', '815', '818', '820', '821', '828', '836', '839', '842', '843', '848', '850', '854', '856', '858', '860', '864', '888', '889', '891', '894', '902', '906', '908', '913', '916', '920', '924', '930', '941', '944', '953', '956', '961', '969', '983', '986', '989', '990', '991', '997', '998', '1007', '1017', '1025', '1029', '1032', '1034', '1035', '1036', '1040', '1042', '1051', '1053', '1054', '1058', '1060', '1061', '1062', '1065', '1076', '1078', '1082', '1085', '1091', '1117', '1120', '1135', '1140', '1145', '1147', '1149', '1150', '1154', '1158', '1159', '1163', '1166', '1170', '1173', '1202', '1203', '1209', '1215', '1218', '1221', '1227', '1234', '1235', '1250', '1259', '1260', '1261', '1262', '1265', '1267', '1291', '1296', '1302', '1303', '1307', '1318', '1322', '1325', '1333', '1342', '1343', '1346', '1366', '1374', '1375', '1378', '1380', '1386', '1413', '1416', '1422', '1423', '1425', '1431', '1445', '1448', '1451', '1486', '1487', '1488', '1491', '1492', '1496', '1499', '1503', '1512', '1519', '1523', '1531', '1533', '1539', '1542', '1544', '1548', '1549', '1554', '1568', '1578', '1583', '1584', '1587', '1599', '1601', '1603', '1606', '1609', '1615', '1618', '1626', '1629', '1638', '1641', '1649', '1650']
Hangzhou_30=['2', '4', '5', '7', '8', '12', '18', '20', '21', '22', '25', '28', '31', '32', '34', '39', '40', '47', '50', '65', '66', '68', '70', '72', '79', '84', '91', '93', '96', '97', '98', '106', '107', '110', '111', '114', '116', '117', '129', '132', '135', '140', '142', '147', '148', '151', '153', '161', '162', '164', '168', '172', '174', '175', '179', '180', '181', '190', '199', '203', '205', '206', '210', '211', '222', '223', '224', '226', '230', '231', '232', '240', '243', '244', '248', '249', '251', '252', '255', '259', '261', '262', '263', '265', '267', '268', '269', '270', '273', '274', '275', '279', '282', '297', '301', '308', '309', '310', '313', '317', '321', '324', '327', '328', '333', '336', '337', '339', '342', '345', '353', '354', '358', '367', '368', '370', '371', '374', '378', '379', '380', '381', '383', '385', '386', '390', '392', '394', '402', '404', '405', '407', '418', '420', '424', '426', '427', '430', '436', '437', '439', '440', '443', '450', '452', '458', '460', '461', '464', '465', '474', '475', '487', '489', '491', '494', '498', '499', '502', '505', '506', '512', '517', '519', '520', '521', '532', '535', '536', '538', '540', '541', '547', '548', '552', '553', '556', '558', '559', '560', '561', '578', '581', '590', '591', '593', '595', '596', '598', '603', '606', '612', '613', '615', '621', '623', '624', '625', '627', '631', '637', '638', '641', '642', '643', '647', '650', '651', '663', '664', '665', '669', '673', '674', '675', '689', '694', '700', '704', '705', '706', '709', '711', '713', '714', '718', '722', '724', '727', '728', '730', '735', '737', '739', '740', '745', '748', '749', '752', '761', '764', '765', '768', '769', '774', '780', '781', '783', '788', '789', '793', '796', '799', '803', '809', '810', '811', '812', '818', '822', '824', '826', '829', '830', '834', '839', '840', '843', '846', '848', '849', '854', '856', '862', '863', '865', '873', '874', '875', '881', '884', '886', '887', '889', '891', '894', '895', '908', '909', '911', '912', '914', '918', '919', '923', '928', '934', '935', '936', '939', '940', '942', '943', '952', '956', '957', '962', '963', '975', '978', '988', '1000', '1012', '1013', '1023', '1026', '1028', '1029', '1031', '1036', '1042', '1043', '1046', '1053', '1055', '1056', '1057', '1059', '1062', '1063', '1065', '1068', '1069', '1073', '1076', '1077', '1084', '1086', '1088', '1090', '1096', '1098', '1099', '1100', '1107', '1124', '1126', '1129', '1135', '1140', '1143', '1144', '1146', '1147', '1152', '1156', '1158', '1162', '1163', '1167', '1170', '1174', '1176', '1179', '1184', '1186', '1187', '1191', '1192', '1194', '1195', '1201', '1208', '1218', '1222', '1231', '1232', '1234', '1236', '1237', '1239', '1248', '1249', '1264', '1265', '1271', '1273', '1275', '1280', '1281', '1283', '1284', '1286', '1291', '1292', '1293', '1297', '1303', '1308', '1310', '1314', '1315', '1317', '1319', '1322', '1325', '1327', '1331', '1333', '1338', '1339', '1343', '1346', '1348', '1349', '1350', '1353', '1361', '1362', '1364', '1367', '1373', '1376', '1395', '1396', '1397', '1401', '1403', '1411', '1416', '1423', '1424', '1431', '1435', '1438', '1439', '1441', '1443', '1446', '1449', '1462', '1463', '1465', '1473', '1475', '1476', '1479', '1483', '1486', '1488', '1491', '1496', '1497', '1503', '1504', '1505', '1507', '1508', '1509', '1511', '1513', '1521', '1526', '1527', '1529', '1539', '1542', '1555', '1557', '1558', '1570', '1577', '1578', '1585', '1588', '1604', '1605', '1606', '1608', '1609', '1611', '1612', '1613', '1614', '1617', '1619', '1620', '1624', '1630', '1631', '1633', '1639', '1643', '1644', '1645', '1647', '1652', '1653']
Hangzhou_40=['0', '1', '3', '5', '8', '10', '12', '16', '17', '19', '20', '24', '26', '30', '31', '32', '40', '41', '47', '48', '49', '50', '51', '57', '60', '62', '64', '67', '68', '69', '70', '72', '73', '74', '77', '83', '86', '87', '89', '91', '96', '98', '99', '101', '104', '108', '110', '115', '117', '122', '127', '130', '131', '140', '142', '144', '147', '148', '149', '152', '153', '154', '156', '159', '160', '162', '163', '167', '171', '175', '177', '179', '182', '183', '186', '187', '188', '189', '197', '198', '201', '202', '203', '204', '205', '211', '212', '218', '219', '221', '224', '226', '229', '231', '239', '243', '245', '246', '247', '248', '249', '250', '256', '257', '259', '261', '264', '267', '273', '275', '282', '284', '285', '287', '289', '290', '292', '294', '295', '296', '297', '303', '306', '308', '311', '312', '315', '316', '317', '318', '321', '322', '323', '324', '325', '329', '331', '333', '335', '337', '338', '340', '346', '347', '348', '349', '352', '353', '359', '363', '364', '371', '374', '379', '389', '390', '391', '392', '393', '396', '399', '400', '403', '409', '410', '411', '412', '413', '414', '416', '419', '421', '422', '423', '426', '427', '430', '435', '438', '439', '445', '451', '454', '455', '457', '459', '465', '466', '471', '472', '475', '477', '480', '484', '488', '489', '491', '494', '495', '496', '497', '498', '499', '501', '502', '504', '506', '512', '513', '515', '516', '520', '521', '523', '527', '529', '532', '533', '535', '537', '538', '544', '546', '547', '548', '550', '551', '552', '553', '557', '558', '562', '564', '565', '568', '570', '574', '576', '577', '580', '582', '583', '590', '593', '595', '599', '600', '601', '604', '605', '607', '608', '610', '612', '616', '618', '620', '622', '624', '626', '627', '628', '633', '640', '645', '646', '647', '648', '650', '652', '654', '655', '660', '663', '666', '667', '673', '674', '675', '676', '678', '680', '682', '683', '686', '691', '695', '697', '699', '700', '702', '704', '706', '707', '709', '710', '713', '718', '719', '720', '727', '728', '734', '735', '737', '744', '745', '747', '748', '751', '753', '757', '760', '763', '768', '771', '772', '773', '774', '775', '780', '781', '785', '786', '787', '789', '790', '795', '797', '798', '803', '805', '812', '813', '814', '819', '821', '822', '824', '826', '827', '833', '838', '840', '841', '842', '843', '845', '853', '854', '855', '857', '858', '859', '862', '863', '866', '867', '870', '871', '873', '876', '880', '885', '891', '893', '911', '913', '916', '917', '918', '920', '921', '923', '926', '927', '932', '933', '934', '936', '937', '938', '939', '943', '944', '947', '948', '950', '951', '953', '957', '960', '962', '963', '964', '965', '966', '968', '973', '974', '977', '980', '982', '985', '988', '990', '993', '996', '997', '999', '1000', '1005', '1006', '1011', '1013', '1015', '1023', '1025', '1027', '1028', '1029', '1030', '1031', '1032', '1033', '1037', '1040', '1041', '1042', '1045', '1046', '1049', '1050', '1051', '1052', '1056', '1068', '1070', '1075', '1077', '1078', '1079', '1083', '1091', '1097', '1100', '1101', '1102', '1103', '1108', '1109', '1110', '1115', '1117', '1123', '1124', '1127', '1128', '1131', '1134', '1135', '1138', '1142', '1149', '1151', '1153', '1154', '1157', '1158', '1162', '1164', '1166', '1167', '1168', '1169', '1171', '1172', '1173', '1174', '1176', '1177', '1178', '1182', '1183', '1186', '1187', '1191', '1193', '1196', '1198', '1199', '1202', '1211', '1214', '1215', '1216', '1220', '1225', '1226', '1228', '1229', '1231', '1232', '1239', '1243', '1244', '1246', '1249', '1252', '1253', '1255', '1257', '1258', '1259', '1261', '1271', '1272', '1274', '1277', '1278', '1281', '1282', '1283', '1284', '1289', '1290', '1304', '1306', '1307', '1312', '1313', '1314', '1316', '1317', '1322', '1324', '1325', '1326', '1327', '1331', '1336', '1337', '1339', '1340', '1341', '1343', '1345', '1347', '1350', '1353', '1361', '1363', '1365', '1367', '1369', '1370', '1371', '1378', '1380', '1384', '1389', '1390', '1391', '1393', '1396', '1397', '1398', '1400', '1401', '1405', '1411', '1412', '1413', '1417', '1418', '1420', '1421', '1424', '1425', '1427', '1428', '1429', '1430', '1431', '1432', '1435', '1438', '1439', '1447', '1448', '1451', '1457', '1458', '1462', '1467', '1470', '1472', '1473', '1474', '1477', '1478', '1479', '1480', '1482', '1483', '1484', '1485', '1486', '1489', '1495', '1496', '1497', '1498', '1500', '1501', '1502', '1503', '1504', '1508', '1509', '1510', '1520', '1524', '1526', '1532', '1541', '1543', '1544', '1545', '1548', '1550', '1551', '1552', '1554', '1557', '1563', '1571', '1573', '1574', '1575', '1576', '1577', '1578', '1583', '1586', '1588', '1590', '1598', '1601', '1603', '1605', '1613', '1622', '1628', '1631', '1632', '1634', '1635', '1636', '1638', '1642', '1650', '1652', '1657']
Hangzhou_50=['0', '3', '4', '5', '6', '7', '8', '10', '12', '13', '14', '15', '16', '18', '19', '20', '21', '25', '26', '28', '31', '32', '33', '34', '38', '40', '42', '43', '45', '47', '53', '54', '56', '60', '61', '62', '63', '66', '68', '70', '72', '74', '75', '76', '77', '79', '80', '81', '82', '84', '85', '86', '90', '95', '96', '97', '99', '100', '106', '109', '110', '112', '113', '118', '120', '121', '123', '124', '127', '129', '131', '132', '133', '136', '140', '141', '142', '143', '145', '146', '148', '150', '157', '158', '162', '167', '169', '170', '172', '173', '174', '176', '177', '180', '181', '182', '187', '188', '193', '197', '198', '200', '202', '204', '206', '207', '208', '211', '212', '213', '214', '215', '216', '218', '220', '221', '223', '224', '226', '228', '230', '231', '234', '237', '238', '240', '243', '244', '245', '246', '248', '250', '257', '259', '260', '262', '263', '264', '265', '268', '271', '272', '273', '276', '279', '280', '282', '284', '285', '287', '288', '292', '293', '295', '296', '297', '298', '301', '307', '308', '309', '311', '313', '315', '319', '321', '322', '323', '324', '328', '330', '331', '335', '336', '337', '340', '341', '342', '347', '350', '351', '352', '354', '357', '359', '361', '362', '363', '365', '371', '372', '373', '374', '382', '383', '385', '388', '390', '391', '392', '396', '397', '401', '402', '403', '404', '408', '411', '412', '413', '417', '418', '420', '421', '431', '433', '434', '436', '437', '438', '442', '443', '445', '446', '448', '449', '450', '451', '454', '456', '458', '459', '461', '467', '468', '469', '470', '471', '472', '474', '475', '476', '477', '478', '481', '485', '487', '488', '491', '493', '494', '495', '496', '497', '498', '499', '500', '501', '506', '507', '508', '511', '514', '515', '516', '518', '520', '522', '523', '526', '527', '528', '529', '531', '534', '535', '536', '537', '538', '540', '541', '544', '545', '546', '550', '551', '552', '555', '557', '559', '562', '564', '565', '566', '567', '568', '570', '575', '576', '580', '581', '582', '585', '586', '587', '588', '591', '593', '596', '600', '601', '602', '603', '604', '609', '611', '613', '616', '617', '619', '620', '622', '624', '628', '629', '631', '634', '635', '636', '637', '638', '639', '640', '643', '644', '645', '647', '648', '651', '652', '656', '658', '659', '660', '661', '662', '663', '665', '666', '667', '668', '669', '670', '671', '672', '673', '674', '678', '682', '684', '687', '690', '692', '693', '696', '697', '699', '700', '702', '706', '709', '710', '711', '712', '713', '714', '716', '717', '722', '723', '724', '728', '729', '730', '731', '735', '736', '737', '743', '748', '749', '750', '753', '754', '755', '756', '758', '759', '760', '761', '762', '763', '764', '765', '767', '768', '769', '771', '773', '774', '775', '778', '781', '783', '784', '789', '792', '793', '794', '796', '797', '799', '800', '803', '805', '807', '809', '811', '813', '816', '818', '821', '824', '825', '826', '831', '834', '837', '838', '840', '841', '843', '848', '850', '853', '854', '856', '857', '858', '862', '863', '865', '866', '867', '868', '869', '871', '872', '873', '875', '877', '880', '881', '882', '883', '887', '888', '889', '892', '896', '899', '901', '902', '907', '908', '909', '910', '912', '914', '916', '920', '921', '923', '925', '928', '929', '930', '932', '934', '937', '939', '941', '944', '945', '948', '950', '951', '953', '956', '957', '958', '960', '962', '964', '965', '966', '967', '968', '969', '972', '975', '979', '982', '983', '984', '986', '989', '991', '998', '999', '1009', '1010', '1011', '1012', '1014', '1015', '1016', '1018', '1019', '1020', '1022', '1025', '1031', '1032', '1035', '1038', '1039', '1040', '1043', '1051', '1052', '1057', '1061', '1065', '1067', '1070', '1071', '1072', '1074', '1077', '1081', '1082', '1083', '1084', '1088', '1092', '1094', '1100', '1101', '1102', '1104', '1107', '1109', '1112', '1113', '1115', '1116', '1122', '1123', '1125', '1126', '1128', '1130', '1131', '1133', '1134', '1136', '1139', '1140', '1141', '1143', '1145', '1152', '1154', '1155', '1157', '1158', '1159', '1160', '1161', '1164', '1165', '1168', '1170', '1174', '1175', '1176', '1177', '1181', '1183', '1184', '1185', '1186', '1187', '1191', '1194', '1195', '1196', '1198', '1199', '1201', '1202', '1203', '1204', '1207', '1208', '1209', '1211', '1212', '1216', '1219', '1220', '1221', '1222', '1224', '1226', '1227', '1235', '1236', '1238', '1239', '1240', '1244', '1246', '1250', '1251', '1252', '1255', '1257', '1259', '1260', '1262', '1265', '1266', '1267', '1270', '1272', '1273', '1274', '1276', '1277', '1278', '1279', '1280', '1282', '1285', '1286', '1288', '1289', '1292', '1293', '1294', '1295', '1298', '1299', '1301', '1303', '1307', '1308', '1313', '1314', '1317', '1319', '1321', '1324', '1329', '1330', '1332', '1334', '1337', '1340', '1343', '1344', '1346', '1348', '1350', '1351', '1353', '1359', '1365', '1367', '1368', '1370', '1371', '1372', '1373', '1374', '1375', '1378', '1380', '1384', '1386', '1389', '1390', '1391', '1392', '1394', '1398', '1400', '1405', '1406', '1407', '1410', '1411', '1412', '1416', '1417', '1419', '1424', '1425', '1431', '1432', '1433', '1437', '1440', '1441', '1448', '1449', '1450', '1454', '1457', '1458', '1460', '1461', '1465', '1466', '1468', '1469', '1472', '1473', '1476', '1477', '1479', '1480', '1481', '1482', '1485', '1487', '1492', '1493', '1501', '1503', '1505', '1508', '1509', '1510', '1519', '1520', '1522', '1523', '1524', '1526', '1527', '1528', '1529', '1530', '1539', '1541', '1543', '1545', '1548', '1549', '1550', '1551', '1555', '1556', '1557', '1559', '1561', '1562', '1570', '1571', '1573', '1575', '1578', '1579', '1580', '1581', '1583', '1584', '1592', '1593', '1594', '1595', '1596', '1598', '1599', '1603', '1605', '1607', '1608', '1610', '1611', '1614', '1616', '1618', '1620', '1621', '1624', '1625', '1627', '1628', '1634', '1635', '1637', '1639', '1640', '1642', '1643', '1646', '1647', '1650', '1651', '1652', '1653', '1654', '1656', '1657', '1658', '1659']
def extract_routes_by_ids(input_file, output_file, desired_ids):
    # 解析rou.xml文件
    tree = ET.parse(input_file)
    root = tree.getroot()

    # 创建一个新的Element来存储提取的vehicle元素
    new_root = ET.Element('routes')
    i = 0
    # 遍历原始XML中的vehicle元素
    for vehicle in root.findall('vehicle'):
        vid = vehicle.attrib.get('id')
        # 检查当前车辆ID是否在指定的ID列表中
        if vid in desired_ids:
            # 创建一个新的vehicle元素，并设置id和depart属性
            new_vehicle = ET.SubElement(new_root, 'vehicle', {
                'id': f'v{i}',
                'depart': '0',

                
            })
            i += 1
            # 复制route子元素到新vehicle中
            for child in vehicle:
                new_vehicle.append(child)

    # 使用minidom美化输出
    rough_string = ET.tostring(new_root, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    pretty_xml_as_string = reparsed.toprettyxml(indent="    ")

    # 将美化后的XML写入到新的文件中
    with open(output_file, 'w') as file_out:
        file_out.write(pretty_xml_as_string)
rou_grid_path = r'test_data\孙倩\grid4x4\grid4x4.rou.xml'
grid_5_output_file = 'grid_5_vechle.xml'
grid_10_output_file = 'grid_10_vechle.xml'
grid_20_output_file = 'grid_20_vechle.xml'
grid_30_output_file = 'grid_30_vechle.xml'
grid_40_output_file = 'grid_40_vechle.xml'
grid_50_output_file = 'grid_50_vechle.xml'
extract_routes_by_ids(rou_grid_path, grid_5_output_file, grid_5)
extract_routes_by_ids(rou_grid_path, grid_10_output_file, grid_10)
extract_routes_by_ids(rou_grid_path, grid_20_output_file, grid_20)
extract_routes_by_ids(rou_grid_path, grid_30_output_file, grid_30)
extract_routes_by_ids(rou_grid_path, grid_40_output_file, grid_40)
extract_routes_by_ids(rou_grid_path, grid_50_output_file, grid_50)

# rou_Hangzhou_path = r'test_data\angzhou-4x4\hangzhou.rou.xml'
# Hangzhou_5_output_file = 'Hangzhou_5_vechle.xml'
# Hangzhou_10_output_file = 'Hangzhou_10_vechle.xml'
# Hangzhou_20_output_file = 'Hangzhou_20_vechle.xml'
# Hangzhou_30_output_file = 'Hangzhou_30_vechle.xml'
# Hangzhou_40_output_file = 'Hangzhou_40_vechle.xml'
# Hangzhou_50_output_file = 'Hangzhou_50_vechle.xml'
# extract_routes_by_ids(rou_Hangzhou_path, Hangzhou_5_output_file, Hangzhou_5)
# extract_routes_by_ids(rou_Hangzhou_path, Hangzhou_10_output_file, Hangzhou_10)
# extract_routes_by_ids(rou_Hangzhou_path, Hangzhou_20_output_file, Hangzhou_20)
# extract_routes_by_ids(rou_Hangzhou_path, Hangzhou_30_output_file, Hangzhou_30)
# extract_routes_by_ids(rou_Hangzhou_path, Hangzhou_40_output_file, Hangzhou_40)
# extract_routes_by_ids(rou_Hangzhou_path, Hangzhou_50_output_file, Hangzhou_50)
