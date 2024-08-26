from utils import  is_nearby_in_map, read_and_visualize_graph, read_graph_json, save_graph_json

graph = read_graph_json()
read_and_visualize_graph(on_map=True,catgeories=["table","chair", "door"])
newposes = [[-22,-39,70],[2.928187961254619,
                -0.47365007929870867,
                0.17512989408567883]]
newclasses = ["door", "table"]
poses = {'door':[], 'chair':[], 'table':[]}
for node, data in graph.nodes(data=True):
    poses[data["category"]].append(data["pose"])

for index, pose in enumerate(newposes):
    poses[newclasses[index]], is_nearby = is_nearby_in_map(poses[newclasses[index]], pose)
    if not is_nearby:
        print(f"adding node")
        graph.add_node(
            f"{newclasses[index]}_new_{index}",
            id=f"{newclasses[index]}_new_{index}",
            pose = pose,
            robot_pose = None,
            category = newclasses[index]
        )

read_and_visualize_graph(on_map=True,catgeories=["table","chair", "door"], graph=graph)
save_graph_json(graph, file="graph_updated.json")


