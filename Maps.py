import folium
import numpy as np
import networkx as nx
import osmnx as ox
import streamlit as st
import streamlit_folium as st_folium
import heapq

# Hàm tìm node trong đồ thị có tọa độ gần với tọa độ click nhất 
def nearest_node(G, x, y):
    # x:longitude, y: latitude
    node_coords = np.array([[G.nodes[node]['x'], G.nodes[node]['y']] for node in G.nodes()])
    target_coords = np.array([x, y])

    # Khoảng cách Eculid của các node trong đồ thị tới vị trí click
    dist = np.sum((node_coords - target_coords) ** 2, axis=1)

    # Vị trí của node gần nhất trong danh sách dist
    nearest_node_index = np.argmin(dist)

    # Lấy id node gần nhất
    nearest_node_id = list(G.nodes())[nearest_node_index]

    return nearest_node_id

# Hàm tìm đường đi ngắn nhất
def shortest_path(graph, start, des, reversed_edges):
    # Khởi tạo các biến lưu khoảng cách và đỉnh trước đó
    distances = {node: float('inf') for node in graph.nodes()}
    previous_nodes = {node: None for node in graph.nodes()}
    distances[start] = 0

    # Tạo một hàng đợi ưu tiên để lựa chọn đỉnh có khoảng cách nhỏ nhất
    priority_queue = [(0, start)]

    while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)

        # Nếu đã tìm được đỉnh đích, dừng thuật toán
        if current_node == des:
            break

        # Duyệt qua các đỉnh kề của đỉnh hiện tại
        for neighbor in graph[current_node]:
            # Lấy trạng thái đường một chiều (oneway) và trạng thái đoạn đường đã bị đảo ngược (reversed) (nếu có)
            oneway = graph[current_node][neighbor][0]['oneway']
            reversed_road = graph[current_node][neighbor][0]['reversed']

            # Lấy trọng số của cạnh giữa đỉnh hiện tại và đỉnh kề
            edge_weight = graph[current_node][neighbor][0]['length']

            # Xét đến đường một chiều và chiều đi là chiều ngược
            if oneway == True and reversed_road == True:
                continue

            # Xét đến đoạn đường hai chiều, hoặc một chiều và chiều đi là chiều thuận
            else:
                new_distance = distances[current_node] + edge_weight

            # Nếu khoảng cách mới nhỏ hơn khoảng cách hiện tại đến đỉnh kề
            if new_distance < distances[neighbor]:
                distances[neighbor] = new_distance
                previous_nodes[neighbor] = current_node
                heapq.heappush(priority_queue, (new_distance, neighbor))

        # Duyệt đỉnh kề ở những đường một chiều nhưng bị đảo ngược
        for edge in list(reversed_edges):  
            if current_node in edge:
                # current_node = edge[0], neighbor = edge[1]
                new_distance = distances[current_node] + reversed_edges[edge[0], edge[1]]
                if new_distance < distances[edge[1]]:
                    distances[edge[1]] = new_distance
                    previous_nodes[edge[1]] = current_node
                    heapq.heappush(priority_queue, (new_distance, edge[1]))

    # Kiểm tra nếu không tìm được đường đi đến điểm đích
    if previous_nodes[des] is None:
        return None

    # Khôi phục đường đi ngắn nhất
    path = []
    current_node = des
    while current_node is not None:
        path.insert(0, current_node)
        current_node = previous_nodes[current_node]

    return path

#Tạo webapp
st.title("Tìm đường đi ngắn nhất trên địa bàn quận Hoàn Kiếm - Hà Nội")
st.write("Nhấp vào bản đồ và chọn nút bên dưới bản đồ để chọn các điểm.\nSau khi chọn điểm xuất phát và điểm đích thì chọn nút 'Tìm đường đi'.")

# Tạo đồ thị chứa các nodes, edges của bản đồ
graph = ox.graph_from_place('Hoan Kiem District, Hanoi, Vietnam', network_type='drive')

# Tạo map
hanoi_map = folium.Map(location=[21.0285, 105.8542], zoom_start=15)

# Tạo popup tại vị trí nhấp
folium.LatLngPopup().add_to(hanoi_map)

#hiển thị bản đồ, st_data lưu tọa độ click
st_data = st_folium.st_folium(hanoi_map)

# Dict lưu trữ key: node id, value: (lat, lng)
node_id_mapping = {}
for n in graph.nodes():
    node_id_mapping[n] = (graph.nodes[n]['y'], graph.nodes[n]['x'])

# Dict lưu trữ các đường một chiều bị đảo ngược so với thực tế. Key: (source_node, target_node), value: length
reversed_edges = {}
edges_list = list(graph.edges(data=True))
for edge in edges_list:
    if edge[2]['oneway'] == True and edge[2]['reversed'] == True:
        reversed_edges[edge[1], edge[0]] = edge[2]['length']

try:
    # Đọc dữ dữ liệu từ bản đồ tương tác
    if st_data is not None and st_data['last_clicked'] is not None:
        # print('lat:', st_data['last_clicked']['lat'], 'lng:', st_data['last_clicked']['lng'])
        lat = st_data['last_clicked']['lat']
        lng = st_data['last_clicked']['lng']

    # Nút chọn điểm xuất phát, ghi tọa độ vào file start.txt
    if st.button("Đây là điểm xuất phát", key="button1"):
        start_point = np.array([lat, lng])
        st.write('Điểm xuất phát: lat: ', start_point[0], '  lng: ', start_point[1])
        f1 = open('start.txt', 'w')
        f1.write("{}\n{}".format(lat,lng))
        f1.close()

    # Nút chọn điểm đích, ghi tọa độ vào file destination.txt
    if st.button("Đây là điểm đích", key="button2"):
        des_point = np.array([lat, lng])
        st.write('Điểm đích: lat: ', des_point[0], '  lng: ', des_point[1])
        f1 = open('destination.txt', 'w')
        f1.write("{}\n{}".format(lat,lng))
        f1.close()

    # Nút tìm đường
    if st.button("Tìm đường đi", key="button3"):
        # Mở file để đọc dữ liệu với con trỏ ở đầu file
        f1 = open('start.txt', 'a+')
        f2 = open('destination.txt', 'a+')
        f1.seek(0)
        f2.seek(0)
        
        #Tạo mảng lưu trữ tọa độ [lat, lng] của 2 điểm mà người dùng nhấp chọn
        start_coordinates = [float(s.strip()) for s in f1.readlines()]
        des_coordinates = [float(s.strip()) for s in f2.readlines()]

        # Đưa ra cảnh báo với người dùng nếu thiếu thông tin
        if (start_coordinates == [] or des_coordinates == []):
            if (start_coordinates == []): st.warning('Bạn cần chọn điểm xuất phát!', icon="⚠️")
            if (des_coordinates == []): st.warning('Bạn cần chọn điểm đích!', icon="⚠️")
        else:
            # Tìm node gần nhất với vị trí mà người dùng click
            start_nearest_node = nearest_node(graph, start_coordinates[1], start_coordinates[0])
            des_nearest_node = nearest_node(graph, des_coordinates[1], des_coordinates[0])

            # Tìm đường đi ngắn nhất
            path = shortest_path(graph, start_nearest_node, des_nearest_node, reversed_edges)

            # Hiển thị kết quả
            if path is None:
                st.write("Không có đường đi giữa hai điểm bạn nhập vào. Có thể do dữ liệu của tôi chưa đủ lớn, xin lỗi vì không đáp ứng được kỳ vọng của bạn!")
            else:
                route = ox.plot_route_folium(graph, path, route_map=folium.Map(), color="#cc0000", weight=5, opacity=0.7)
                folium.PolyLine([tuple(start_coordinates), (graph.nodes[start_nearest_node]['y'], graph.nodes[start_nearest_node]['x'])],color="#cc0000", weight=5, opacity=0.7).add_to(route)
                folium.PolyLine([tuple(des_coordinates), (graph.nodes[des_nearest_node]['y'], graph.nodes[des_nearest_node]['x'])],color="#cc0000", weight=5, opacity=0.7).add_to(route)
                folium.Marker(
                    start_coordinates,
                    popup='Origin',
                    icon=folium.Icon(color="red", icon="1", prefix='fa')
                ).add_to(route)
                folium.Marker(
                    des_coordinates,
                    popup='Destination',
                    icon=folium.Icon(color="blue", icon="2", prefix='fa')
                ).add_to(route)
                st.write("Marker màu đỏ là vị trí xuất phát của bạn.")
                st.write("Marker màu xanh là đích đến của bạn.")
                st.write("Hiện tại dữ liệu của tôi vẫn còn nhỏ. Vì vậy khi chọn vào vị trí ngoài vùng dữ liệu, đường đi sẽ được nối thẳng từ vị trí được chọn đến điểm gần nhất trong vùng dữ liệu. Xin lỗi vì sự bất tiện này!")
                st_folium.folium_static(route)

            # làm rỗng file start.txt và destination.txt
            f1.seek(0)
            f1.truncate(0)
            f2.seek(0)
            f2.truncate(0)
        f1.close()
        f2.close()
except TypeError:
    pass
