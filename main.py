from utils import *
# Generate and visualization g raph
plt.figure(figsize=(16, 13))
graph = nx.Graph()
n_ver = 100
m_ed = 500
graph.add_nodes_from(range(n_ver))
for pair in random.sample([*permutations(range(n_ver), 2)], m_ed):
    graph.add_edge(*pair, weight=random.randint(10, 100))
    nx.draw(graph, node_size=100, node_color='grey', edge_color='grey')
plt.show()

mtx = np.array(nx.adjacency_matrix(graph).todense())
np.set_printoptions(edgeitems=6, linewidth=150)



 #A*
cells = np.zeros((10, 20))
# Creating a list of elements positions
positions = []
for i in range(10):
    for j in list(range(k, 20)):
        pos = [i, j]
        positions.append(pos)
position_random = positions.copy()
random.shuffle(position_random)
# choose 40 random elements
pos_40 = position_random[:40]
# replace elements on positions that were chosen to declare the edge
for i in pos_40:
    k = i[0]
    m = i[1]
    cells[k][m] = 1
pos_non_obstance = []
pos_non_obstance_x = []
pos_non_obstance_y = []

for i in range(10):
    for j in list(range(k, 20)):
        if cells[i][j] != 1:
            pos_non_obstance.append([i, j])
            pos_non_obstance_x.append(i)
            pos_non_obstance_y.append(j)
start_pos = random.choice(pos_non_obstance)
end_pos = random.choice(pos_non_obstance)


#RESULT
path=astar(cells, tuple(start_pos),tuple(end_pos), allow_diagonal_movement = False)

x_path = []
y_path = []
for i in path:
    x_path.append(i[0])
    y_path.append(i[1])

i = 10
j = 20
obstacles = 40

gg = nx.grid_graph(dim=[i, j])
gg = delete_node(gg, obstacles)

a = random.choice(list(gg.nodes))
b = random.choice(list(gg.nodes))
path = nx.algorithms.shortest_paths.astar.astar_path(gg, start, target)


print('path between %s and %s includes %i cells' %
        ((4,11), (2,15), len(path)-2))
print('path - %s' % path)
plt.figure(figsize=(10,8))
plt.rcParams.update({'font.size': 10})
plt.scatter(y,x, c = 'orange', s = 150, label = 'Obstance cells')
plt.scatter(pos_non_obstance_y,pos_non_obstance_x, c ='gray', s = 150)
plt.scatter(y_path,x_path,c = 'black', s = 250, label = 'Path')
plt.scatter(start_pos[1],start_pos[0], c = 'blue', s = 200, label = 'Start point')
plt.scatter(end_pos[1],end_pos[0], c = 'red', s = 200, label = 'End point')
plt.plot(y_path,x_path, color='black')
plt.legend(loc = 'best')
plt.show()
#Plot
plt.figure(figsize=(10,8))
plt.rcParams.update({'font.size': 10})
plt.scatter(y,x, c = 'orange', s = 150, label = 'Obstance cells')
plt.scatter(pos_non_obstance_y,pos_non_obstance_x, c ='gray', s = 150)
plt.scatter(y_path,x_path,c = 'black', s = 250,
label = 'Path')
plt.scatter(start_pos[1],start_pos[0], c = 'blue', s = 200,
label = 'Start point')
plt.scatter(end_pos[1],end_pos[0], c = 'red', s = 200,
label = 'End point')
plt.plot(y_path,x_path, color='black')
plt.legend(loc = 'best')
plt.show()

def A_analysis(cell_grid,n_iterations,non_obstacle):
    a_star_time = []
    for i in range(n_iterations):
        #choose 2 random non-obstacle cells
        cell_random = non_obstacle.copy()
        random.shuffle(cell_random)
        two_cell = cell_random[:2]
        #calculate time for finding the shortest path
        start_time_A = perf_counter()
        path = astar(cell_grid, two_cell[0], two_cell[1], allow_diagonal_movement = False)
        end_time_A=perf_counter()
        a_star_time.append(end_time_A-start_time_A)
        print('path between %s and %s includes %i cells' %
        (two_cell[0], two_cell[1],len(path)-2))
        print('path - %s' % path)
    print('average time for the paths search - %f sec' % np.mean(a_star_time))
A_analysis(cells,10,pos_non_obstance)


