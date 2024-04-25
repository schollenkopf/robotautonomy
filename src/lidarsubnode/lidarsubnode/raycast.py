from numpy import cos,sin,pi
import numpy as np
import math


def bresenham(start, end) -> np.ndarray:
    """
    Implementation of Bresenham's line drawing algorithm
    See en.wikipedia.org/wiki/Bresenham's_line_algorithm
    Bresenham's Line Algorithm
    Produces a np.array from start and end including.

    (original from roguebasin.com)
    >> points1 = bresenham((4, 4), (6, 10))
    >> print(points1)
    np.array([[4,4], [4,5], [5,6], [5,7], [5,8], [6,9], [6,10]])
    """
    # setup initial conditions
    x1, y1 = start
    x2, y2 = end
    dx = x2 - x1
    dy = y2 - y1
    is_steep = abs(dy) > abs(dx)  # determine how steep the line is
    if is_steep:  # rotate line
        x1, y1 = y1, x1
        x2, y2 = y2, x2
    # swap start and end points if necessary and store swap state
    swapped = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True
    dx = x2 - x1  # recalculate differentials
    dy = y2 - y1  # recalculate differentials
    error = int(dx / 2.0)  # calculate error
    y_step = 1 if y1 < y2 else -1
    # iterate over bounding box generating points between start and end
    y = y1
    points = []
    for x in range(x1, x2 + 1):
        coord = [y, x] if is_steep else (x, y)
        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += y_step
            error += dx
    if swapped:  # reverse the list if the coordinates were swapped
        points.reverse()
    points = np.array(points)
    return points

def ray_cast(r,a,x_index,y_index,cell_size):
    x_d = r*cos(a)
    y_d = r*sin(a)
    map_x = int(x_index+x_d/cell_size)
    map_y = int(y_index+y_d/cell_size)
    return bresenham((int(x_index),int(y_index)), (map_x,map_y))


def ray_cast_gain(map,x_coord,y_coord,cell_size,max_range=3.5) -> int:
    unknown_cells = set()
    rad_step = (2*pi)/360
    x_index = int((x_coord)/cell_size)+100
    y_index = int((y_coord)/cell_size)+100
    a = 0
    for i in range(0, 360):
        a = i * rad_step
        x_d = max_range*cos(a)
        y_d = max_range*sin(a)
        end_x_index = int((x_coord+x_d)/cell_size)+100
        end_y_index = int((y_coord+y_d)/cell_size)+100
        ray = bresenham((x_index,y_index), (end_x_index,end_y_index))
        for x,y in ray:
            if x >= 200 or y >= 200 or x < 0 or y < 0:
                break
            if map[x,y] > 20:
                break
            if map[x,y] == -1:
                unknown_cells.add((x,y))
    return len(unknown_cells)

def ray_cast_gain2(map,x_coord,y_coord,cell_size,max_range=3.5):
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
                print("-1 at:",x_index,y_index)
                gain += 1
        
    return gain