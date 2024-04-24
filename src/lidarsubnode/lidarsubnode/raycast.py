from math import ceil
from numpy import cos,sin,pi

def ray_cast(r,a,curr_x,curr_y,cell_size):
    step_size = cell_size/2
    cells = []
    t = 0
    while t < r:
        t = t + step_size
        x = t*cos(a)
        y = t*sin(a)
        map_x = int(curr_x+x/cell_size)
        map_y = int(curr_y+y/cell_size)
        cells.append((map_x,map_y))
    return cells

def ray_cast_gain(map,curr_x,curr_y,cell_size,max_range=3.5):
    step_size = cell_size/2
    gain = 0
    t = 0
    a = 0
    rad_step = (2*pi)/360
    for i in range(0,360):
        a = a+rad_step
        while t < max_range:
            t = t + step_size
            x = t*cos(a)
            y = t*sin(a)
            map_x = int((curr_x+x)/cell_size)+100
            map_y = int((curr_y+y)/cell_size)+100
            if map_x >= 200 or map_y >= 200 and map_x < 0 and map_y < 0:
                break
            if map[map_x,map_y] > 20 :
                break
            if map[map_x,map_y] == -1:
                gain += 1
        
    return gain