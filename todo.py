import heapq
import networkx as nx
import matplotlib.pyplot as plt

def astar(graph, start, goal):
    open_set = []
    closed_set = set()
    came_from = {}
    g_score = {node: float('inf') for node in graph}
    g_score[start] = 0
    f_score = {node: float('inf') for node in graph}
    f_score[start] = heuristic(start, goal)

    heapq.heappush(open_set, (f_score[start], start))

    while open_set:
        current = heapq.heappop(open_set)[1]

        if current == goal:
            return reconstruct_path(came_from, start, goal)

        closed_set.add(current)

        for neighbor in graph[current]:
            if neighbor in closed_set:
                continue

            tentative_g_score = g_score[current] + graph[current][neighbor]

            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)

                if neighbor not in open_set:
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None  # No path found

def reconstruct_path(came_from, start, goal):
    if goal not in came_from:
        return None  # No path found

    path = [goal]
    while goal != start:
        goal = came_from[goal]
        path.append(goal)
    return path[::-1]

def heuristic(node, goal):
    # Implementa tu heurística aquí (puede ser la distancia euclidiana, por ejemplo)
    return 0

# Función para visualizar el grafo y la ruta
def visualize_graph(graph, path):
    G = nx.Graph(graph)
    pos = nx.spring_layout(G)  # Posiciones para los nodos

    nx.draw(G, pos, with_labels=True, font_weight='bold', node_color='lightblue', font_color='black')

    # Dibuja las etiquetas de los pesos de las aristas
    edge_labels = {(i, j): str(weight) for i, j, weight in G.edges(data='weight')}
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)

    # Resalta los nodos de la ruta
    if path and path[0] in G.nodes:
        path_edges = [(path[i], path[i + 1]) for i in range(len(path) - 1)]
        nx.draw_networkx_nodes(G, pos, nodelist=path, node_color='red')
        nx.draw_networkx_edges(G, pos, edgelist=path_edges, edge_color='red', width=2)

    plt.show()

# Ejemplo de uso
graph = {
    'A': {'B': 1, 'C': 4},
    'B': {'A': 1, 'C': 2, 'D': 5},
    'C': {'A': 4, 'B': 2, 'D': 1},
    'D': {'B': 5, 'C': 1}
}

start_node = 'A'
goal_node = 'D'  # No existe un nodo 'E' en este ejemplo, debería devolver None
path = astar(graph, start_node, goal_node)

if path is not None:
    print("Shortest path:", path)
    visualize_graph(graph, path)
else:
    print("No path found.")
