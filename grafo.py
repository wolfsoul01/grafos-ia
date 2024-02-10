import networkx as nx
import matplotlib.pyplot as plt

G = nx.DiGraph()
G.add_nodes_from([1, 2, 3, 4, 5])

G.add_weighted_edges_from([(1, 2, 2), (1, 3, 1), (2, 4, 3), (3, 4, 4), (4, 5, 5)])

# Realizar un recorrido en profundidad (DFS)
dfs_edges = list(nx.dfs_edges(G, source=1))

pos = nx.spring_layout(G, seed=42)

nx.draw(G, pos, with_labels=True, font_weight='bold', node_size=700, node_color='skyblue', font_color='black')

# Agregar etiquetas de peso en las aristas
edge_labels = {(u, v): G[u][v]['weight'] for u, v in G.edges}
nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_color='red')

# Resaltar el recorrido en profundidad en rojo
nx.draw_networkx_edges(G, pos, edgelist=dfs_edges, edge_color='r', width=2)

plt.show()
