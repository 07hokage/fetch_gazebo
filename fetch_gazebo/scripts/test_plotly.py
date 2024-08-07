import networkx as nx
import plotly.graph_objects as go

# Create a NetworkX graph
G = nx.Graph()

# Add nodes with attributes
nodes = {
    "Table": {"category": "Furniture", "description": "A piece of furniture with a flat top."},
    "Chair": {"category": "Furniture", "description": "A piece of furniture designed for sitting."},
    "Door": {"category": "Building Element", "description": "A hinged or sliding panel used to close off an entrance."},
    "Table1": {"category": "Furniture", "description": "Wooden table"},
    "Table2": {"category": "Furniture", "description": "Glass table"},
    "Table3": {"category": "Furniture", "description": "Metal table"},
    "Chair1": {"category": "Furniture", "description": "Office Chair"},
    "Chair2": {"category": "Furniture", "description": "Dining Chair"},
    "Door1": {"category": "Building Element", "description": "Sliding door"},
    "Door2": {"category": "Building Element", "description": "Hinged door"},
    "Door3": {"category": "Building Element", "description": "Revolving door"}
}

# Add nodes with attributes to the graph
for node, attr in nodes.items():
    G.add_node(node, **attr)

# Add edges (both hierarchical and associative)
edges = [
    ("Table", "Table1"),
    ("Table", "Table2"),
    ("Table", "Table3"),
    ("Chair", "Chair1"),
    ("Chair", "Chair2"),
    ("Door", "Door1"),
    ("Door", "Door2"),
    ("Door", "Door3"),
    ("Table", "Chair"),
    ("Table", "Door"),
    ("Chair", "Door")
]

# Add edges to the graph
G.add_edges_from(edges)

# Define custom 3D positions
pos = {
    "Table": (0, 0, 5),
    "Chair": (10, 0, 5),
    "Door": (20, 0, 5),
    "Table1": (0, 2, 0),
    "Table2": (0, 4, 0),
    "Table3": (0, 6, 0),
    "Chair1": (10, 2, 0),
    "Chair2": (10, 4, 0),
    "Door1": (20, 2, 0),
    "Door2": (20, 4, 0),
    "Door3": (20, 6, 0)
}

# Extract node positions
x_nodes = [pos[node][0] for node in G.nodes()]
y_nodes = [pos[node][1] for node in G.nodes()]
z_nodes = [pos[node][2] for node in G.nodes()]

# Create edge positions
x_edges = []
y_edges = []
z_edges = []
for edge in G.edges():
    x_edges.extend([pos[edge[0]][0], pos[edge[1]][0], None])
    y_edges.extend([pos[edge[0]][1], pos[edge[1]][1], None])
    z_edges.extend([pos[edge[0]][2], pos[edge[1]][2], None])

# Create the figure
fig = go.Figure()

# Add nodes
fig.add_trace(go.Scatter3d(
    x=x_nodes,
    y=y_nodes,
    z=z_nodes,
    mode='markers+text',
    text=[node for node in G.nodes()],
    marker=dict(size=10, color='blue'),
    textposition='top center'
))

# Add edges
fig.add_trace(go.Scatter3d(
    x=x_edges,
    y=y_edges,
    z=z_edges,
    mode='lines',
    line=dict(width=2, color='grey')
))

# Set layout
fig.update_layout(
    title="3D Visualization of NetworkX Graph",
    scene=dict(
        xaxis_title='X Axis',
        yaxis_title='Y Axis',
        zaxis_title='Z Axis'
    ),
    showlegend=False
)

# Show the figure
fig.show()
