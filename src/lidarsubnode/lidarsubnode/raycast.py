from math import ceil
from numpy import cos,sin,pi

def ray_cast(r,a,curr_x,curr_y,cell_size):
    step_size = cell_size/2
    cells = []
    for t in range(ceil(r/step_size)):
        t = t * step_size
        x = t*cos(a)
        y = t*sin(a)
        map_x = int(curr_x+x/cell_size)
        map_y = int(curr_y+y/cell_size)
        cells.append((map_x,map_y))
    return cells

def ray_cast_gain(map,curr_x,curr_y,cell_size,max_range=3.5):
    step_size = cell_size/2
    gain = 0
    a = 0
    rad_step = (2*pi)/360
    for i in range(0,360):
        a = a+rad_step
        for t in range(ceil(max_range/step_size)):
            t = t * step_size
            x = t*cos(a)
            y = t*sin(a)
            map_x = int(curr_x+x/cell_size)
            map_y = int(curr_y+y/cell_size)
            
            if map[map_x,map_y] > 0 :
                break
            if map[map_x,map_y] == -1:
                gain += 1
    return gain