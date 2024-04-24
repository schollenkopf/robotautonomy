from math import ceil
from numpy import cos,sin,pi

def ray_cast(r,a,x_index,y_index,cell_size):
    step_size = cell_size/2
    cells = []
    t = 0
    while t < r:
        t = t + step_size
        x_d = t*cos(a)
        y_d = t*sin(a)
        map_x = int(x_index+x_d/cell_size)
        map_y = int(y_index+y_d/cell_size)
        if not (map_x >= 200 or map_y >= 200 or map_x < 0 or map_y < 0):
            cells.append((map_x,map_y))
        
    return cells

def ray_cast_gain(map,x_coord,y_coord,cell_size,max_range=3.5):
    step_size = cell_size/2
    gain = 0
    t = 0
    a = 0
    rad_step = (2*pi)/360
    for i in range(0,360):
        a = a+rad_step
        while t < max_range:
            t = t + step_size
            x_d = t*cos(a)
            y_d = t*sin(a)
            x_index = int((x_coord+x_d)/cell_size)+100
            y_index = int((y_coord+y_d)/cell_size)+100
            if x_index >= 200 or y_index >= 200 or x_index < 0 or y_index < 0:
                break
            if map[x_index,y_index] > 20:
                break
            if map[x_index,y_index] == -1:
                gain += 1
        
    return gain