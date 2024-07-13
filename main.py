import heapq

import plotly.graph_objs as go
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import plotly.graph_objs as go


def write_to_file(pathx,pathy,cost):
    with open('output.txt', 'w') as f:
        f.write(f'Koltseg: {cost}\n')
        f.write('Utvonal:\n')
        for i in range(len(pathx)):
            f.write(f'{pathx[i]}, {pathy[i]}\n')

def write_to_file2(pathx,pathy,cost):
    with open('output_2.txt', 'w') as f:
        f.write(f'Koltseg: {cost}\n')
        f.write('Utvonal:\n')
        for i in range(len(pathx)):
            f.write(f'{pathx[i]}, {pathy[i]}\n')
def dist(coords, index1, index2):
    x1, y1, z1 = coords[0][index1], coords[1][index1], coords[2][index1]
    x2, y2, z2 = coords[0][index2], coords[1][index2], coords[2][index2]
    return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)

# def neighbour2(coords, index):
#     x, y = coords[0][index], coords[1][index]
#     neighbours = []
#     for i in range(len(coords[0])):
#         if ((coords[0][i], coords[1][i]) != (x, y)) and (coords[3][i] == 0) and abs(x-coords[0][i])<=1 and abs(y-coords[1][i])<=1:
#             neighbours.append(i)
#     return neighbours

def neighbour(coords, index):
    x, y = coords[0][index], coords[1][index]
    neighbours = []
    step = int(np.sqrt(len(coords[0])))
    new_i = index - 1
    if new_i >= 0 and ((coords[0][new_i], coords[1][new_i]) != (x, y)) and (coords[3][new_i] == 0) and abs(x - coords[0][new_i]) <= 1 and abs(y - coords[1][new_i]) <= 1:
        neighbours.append(new_i)
    new_i = index + 1
    if new_i < len(coords[0]) and ((coords[0][new_i], coords[1][new_i]) != (x, y)) and (coords[3][new_i] == 0) and abs(x - coords[0][new_i]) <= 1 and abs(y - coords[1][new_i]) <= 1:
        neighbours.append(new_i)
    new_i = index - step - 1
    if new_i >= 0 and ((coords[0][new_i], coords[1][new_i]) != (x, y)) and (coords[3][new_i] == 0) and abs(x - coords[0][new_i]) <= 1 and abs(y - coords[1][new_i]) <= 1:
        neighbours.append(new_i)
    new_i = index - step
    if new_i >= 0 and ((coords[0][new_i], coords[1][new_i]) != (x, y)) and (coords[3][new_i] == 0) and abs(x - coords[0][new_i]) <= 1 and abs(y - coords[1][new_i]) <= 1:
        neighbours.append(new_i)
    new_i = index - step + 1
    if new_i < len(coords[0]) and ((coords[0][new_i], coords[1][new_i]) != (x, y)) and (coords[3][new_i] == 0) and abs(x - coords[0][new_i]) <= 1 and abs(y - coords[1][new_i]) <= 1:
        neighbours.append(new_i)
    new_i = index + step - 1
    if new_i >= 0 and new_i<len(coords[0]) and ((coords[0][new_i], coords[1][new_i]) != (x, y)) and (coords[3][new_i] == 0) and abs(x - coords[0][new_i]) <= 1 and abs(y - coords[1][new_i]) <= 1:
        neighbours.append(new_i)
    new_i = index + step
    if new_i < len(coords[0]) and ((coords[0][new_i], coords[1][new_i]) != (x, y)) and (coords[3][new_i] == 0) and abs(x - coords[0][new_i]) <= 1 and abs(y - coords[1][new_i]) <= 1:
        neighbours.append(new_i)
    new_i = index + step + 1
    if new_i < len(coords[0]) and ((coords[0][new_i], coords[1][new_i]) != (x, y)) and (coords[3][new_i] == 0) and abs(x - coords[0][new_i]) <= 1 and abs(y - coords[1][new_i]) <= 1:
        neighbours.append(new_i)
    return neighbours



def a_star_step(coords, start_index, goal_index): #lepes szamlalo
    frontier = []
    heapq.heappush(frontier, (0, start_index))
    came_from = {}
    cost_so_far = {}
    came_from[start_index] = None
    cost_so_far[start_index] = 0

    while len(frontier) > 0:
        current_index = heapq.heappop(frontier)[1]

        if current_index == goal_index:
            break #g = 2pont kozti tav #h = aktualis pont es end kozti tav

        for next_index in neighbour(coords, current_index):
            new_cost = cost_so_far[current_index] + 1 #mindig egy lesz a tav me lepes   #f
            if next_index not in cost_so_far or new_cost < cost_so_far[next_index]:
                cost_so_far[next_index] = new_cost
                priority = new_cost + max(abs(coords[1][current_index] - coords[1][goal_index]), abs(coords[0][current_index] - coords[0][goal_index]))
                heapq.heappush(frontier, (priority, next_index))
                came_from[next_index] = current_index

    current_index = goal_index
    path = []
    while current_index is not None:
        path.append(current_index)
        current_index = came_from[current_index]
    return path
def a_star(coords, start_index, goal_index):#tavolsaggal szamlalo 28
    frontier = []
    heapq.heappush(frontier, (0, start_index))
    came_from = {}
    cost_so_far = {}
    came_from[start_index] = None
    cost_so_far[start_index] = 0

    while len(frontier) > 0:
        current_index = heapq.heappop(frontier)[1]

        if current_index == goal_index:
            break

        for next_index in neighbour(coords, current_index):
            new_cost = cost_so_far[current_index] + dist(coords, current_index, next_index)
            if next_index not in cost_so_far or new_cost < cost_so_far[next_index]:
                cost_so_far[next_index] = new_cost
                priority = new_cost + dist(coords, goal_index, next_index)
                heapq.heappush(frontier, (priority, next_index))
                came_from[next_index] = current_index

    current_index = goal_index
    path = []
    while current_index is not None:
        path.append(current_index)
        current_index = came_from[current_index]

    path_length = sum(dist(coords, path[i], path[i + 1]) for i in range(len(path)-1))

    return path, path_length

file = open("surface_100x100.txt", "r")
#file = open("surface_512x512.txt", "r")

coords = []
x = []
y = []
z = []
xo = []
yo = []
zo = []
X = []
Y = []
Z = []
color = []
distance = 0
for line in file.readlines():
    fields = line.split(' ')
    # print(fields)
    if fields[3] == '1\n':
        xo.append(float(fields[0]))
        yo.append(float(fields[1]))
        zo.append(float(fields[2]))
    else:
        x.append(float(fields[0]))
        y.append(float(fields[1]))
        z.append(float(fields[2]))
    X.append(float(fields[0]))
    Y.append(float(fields[1]))
    Z.append(float(fields[2]))
    color.append(float(fields[3]))
coords.append(X)
coords.append(Y)
coords.append(Z)
coords.append(color)
file.close()

# sx=60.0
# sy=200.0
# ex=450.0
# ey=15.0
#
sx=0.0
sy=0.0
ex=99.0
ey=30.0
s=0
e=0
for i in range(len(coords[0])):
    if coords[0][i] == sx and coords[1][i] == sy:
        s=i
    if coords[0][i] == ex and coords[1][i] == ey:
        e=i

Pathx = []
Pathy = []
Pathz = []
Path2x = []
Path2y = []
Path2z = []

path,distance = a_star(coords,s,e)

print(f"Path: {path}")
print(f"Distance: {distance}")
for i in range(len(path)):
    Pathx.append(coords[0][path[i]])
    Pathy.append(coords[1][path[i]])
    Pathz.append(coords[2][path[i]])

path2=a_star_step(coords,s,e)
print(f"Path: {path2}")
print(f"Distance: {len(path2)-1}")
for i in range(len(path2)):
    Path2x.append(coords[0][path2[i]])
    Path2y.append(coords[1][path2[i]])
    Path2z.append(coords[2][path2[i]])

scatter4 = go.Scatter3d(
    x=Path2x,
    y=Path2y,
    z=Path2z,
    mode='markers',
    marker=dict(
        size=4,
        color='green',
        opacity=1.0
    )
)


scatter3 = go.Scatter3d(
    x=Pathx,
    y=Pathy,
    z=Pathz,
    mode='markers',
    marker=dict(
        size=4,
        color='green',
        opacity=1.0
    )
)

scatter2 = go.Scatter3d(
    x=xo,
    y=yo,
    z=zo,
    mode='markers',
    marker=dict(
        size=4,
        color='red',
        opacity=0.7
    )
)

scatter = go.Scatter3d(
    x=x,
    y=y,
    z=z,
    mode='markers',
    marker=dict(
        size=3,
        color=z,
        opacity=0.3
    )
)

layout = go.Layout(
    title='Utkereses',
    scene=dict(
        xaxis=dict(title='X'),
        yaxis=dict(title='Y'),
        zaxis=dict(title='Z'),
    )
)
#fig = go.Figure(data=[scatter3], layout=layout)
fig = go.Figure(data=[scatter3, scatter, scatter2], layout=layout)
fig2 = go.Figure(data=[scatter4, scatter, scatter2], layout=layout)


fig.show()
fig2.show()
write_to_file(Pathx,Pathy,distance)
write_to_file2(Path2x,Path2y,len(path2)-1)