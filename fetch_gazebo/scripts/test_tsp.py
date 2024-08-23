import networkx as nx
tsp = nx.approximation.traveling_salesman_problem
from utils import read_graph_json, read_map_image, read_map_metadata, pose_to_map_pixel, display_map_image, read_and_visualize_graph
from numpy.linalg import norm
import time
# 
# G = nx.cycle_graph(9)
# print(G)
# G[4][5]["weight"] = 5  # all other weights are 1
# path = tsp(G, nodes=[3, 6])
# print(path, "\n")
# Solve TSP using a greedy heuristic
def tsp_greedy_solution(G):
    nodes = list(G.nodes)
    path = [nodes[0]]  # Start at the first node
    visited = set(path)

    while len(path) < len(nodes):
        last = path[-1]
        next_node = min(
            (n for n in nodes if n not in visited),
            key=lambda n: G[last][n]['weight']
        )
        path.append(next_node)
        visited.add(next_node)

    path.append(path[0])  # Return to the starting node
    return path

graph = read_graph_json()
read_and_visualize_graph(on_map=True, catgeories=["table", "chair", "door"])
# add edges betweeen each nodes
# for id, (node, data) in enumerate(graph.nodes(data=True)):
#     for id_, (node_, data_) in enumerate(graph.nodes(data=True)):
#         if node == node_:
#             continue
#         else:
#             pose1 = data["pose"]
#             pose2 = data_["pose"]
#             weight=norm((pose1[0] - pose2[0], pose1[1] - pose2[1]))
#             if id == id_ - 1:
#                 graph.add_edge(node, node_, weight=weight)
#             elif weight > 4:
#                 graph.add_edge(node, node_, weight=weight)
#             else:
#                 continue

map_image = read_map_image("map.png")
map_metadata = read_map_metadata("map.yaml")



# graph_ = graph.copy()
# for node, data in graph.nodes(data=True):
#     x,y = pose_to_map_pixel(map_metadata, data["pose"])
#     map_image[
#                    y-10// 2 :y+10// 2,
#                     x-10 // 2 : x+10 // 2,
#                     :,
#                 ] = [0,0,255]
#     display_map_image(map_image, write=True)


graph_ = graph.copy()
print(graph_.number_of_nodes())
# time.sleep(5)
# print(graph.nodes())
deleted_noodes = []
for id, (node, data) in enumerate(graph.nodes(data=True)):
    # print(f"main node {node}\n")
    if node in deleted_noodes:
            continue
    for id_, (node_, data_) in enumerate(graph.nodes(data=True)):
        if node_ in deleted_noodes:
            continue
        # print(f"sub noide {node_}")
        # print(data)
        # if node == node_ or (id_ < id):
        if node == node_ or (id_ < id):
            continue
        pose1 = data["pose"]
        pose2 = data_["pose"]
        weight=norm((pose1[0] - pose2[0], pose1[1] - pose2[1]))
        if weight < 10:
            graph_.remove_node(node_)
            deleted_noodes.append(node_)
            # print("removed node ")
        #     x,y = pose_to_map_pixel(map_metadata, data["pose"])
        #     map_image[
        #                 y-10// 2 :y+10// 2,
        #                     x-10 // 2 : x+10 // 2,
        #                     :,
        #                 ] = [0,0,255]
        # display_map_image(map_image, write=True)

for id, (node, data) in enumerate(graph_.nodes(data=True)):
    for id_, (node_, data_) in enumerate(graph_.nodes(data=True)):
        if id < id_:
            pose1 = data["pose"]
            pose2 = data_["pose"]
            weight=norm((pose1[0] - pose2[0], pose1[1] - pose2[1]))
            graph_.add_edge(node, node_, weight=weight)

path = tsp_greedy_solution(graph_)
total_distance = sum(
        graph_[path[i]][path[i + 1]]['weight']
        for i in range(len(path) - 1)
    )
print(total_distance)
path = list(graph_.nodes())
total_distance = sum(
        graph_[path[i]][path[i + 1]]['weight']
        for i in range(len(path) - 1)
    )
print(total_distance)
for node in path:
    pose = graph_.nodes[node]["pose"]
    x,y = pose_to_map_pixel(map_metadata, pose)
    map_image[
                   y-10// 2 :y+10// 2,
                    x-10 // 2 : x+10 // 2,
                    :,
                ] = [0,0,255]
    display_map_image(map_image, write=True)

print(graph_.number_of_nodes())

# path = tsp(graph, cycle=True)
# graph_ =graph.nodes(data=True)
# for node in path:
#     x,y = pose_to_map_pixel(map_metadata, graph_[node]["pose"])
#     map_image[
#                    y-10// 2 :y+10// 2,
#                     x-10 // 2 : x+10 // 2,
#                     :
#                 ] = [0,0,255]
#     display_map_image(map_image, write=True)
#     map_image[
#                    y-10// 2 :y+10// 2,
#                     x-10 // 2 : x+10 // 2,
#                     :,
#                 ] = [255,255,255]
# print(path)