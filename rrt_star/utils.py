""" 
Utilities for collision checking in rrt
@author: patrick musau
Inspired by huiming zhou
"""

import math
import numpy as np
import os
import sys

import env

# import node from rrt_star (x,y,parent)
from rrt_star import Node


class Utils:
    def __init__(self,x_width=50,y_width=30):
        
        # define the environment width environment etc
        self.env = env.Env(x_width,y_width)

        # definitions of obstacles and workspace boundaries
        self.obs_circle = self.env.obs_circle()
        self.obs_rectangle = self.env.obs_rectangle()
        self.obs_boundary = self.env.obs_boundary()

        # my best guess of what this is so far is a tolerance
        # how much to move each point out
        self.delta = 0.5

    
    def update_obs(self, obs_cir, obs_bound, obs_rec):
        self.obs_circle = obs_cir
        self.obs_boundary = obs_bound
        self.obs_rectangle = obs_rec


    # for eah of the obstacles get the list of the vertices 
    # for example if you have a box with vertex at (1,1) with a height and width of 1,1
    # it generates the points (0.5,0.5), (2.5,0.5), 2.5,2.5, 0.5,2.5
    # This function returns  a list of 4 vertices for each rectangle

    # vertex order [bottom_left,bottom_right,top_right,top_left]

    def get_obs_vertex(self):
        delta = self.delta
        obs_list = []

        for (ox, oy, w, h) in self.obs_rectangle:
            vertex_list = [[ox - delta, oy - delta],
                           [ox + w + delta, oy - delta],
                           [ox + w + delta, oy + h + delta],
                           [ox - delta, oy + h + delta]]
            obs_list.append(vertex_list)

        return obs_list


    # Intersection checking for rectangle we will look into this some more but it's
    # essentially checking for an intersection with a box
    # Description
    # - start: start point 
    # - end: end point 
    # - o: origin of ray
    # - d: direction of ray
    # - a: vertex 1
    # - b: vertex 2
    def is_intersect_rec(self, start, end, o, d, a, b):
        v1 = [o[0] - a[0], o[1] - a[1]]
        v2 = [b[0] - a[0], b[1] - a[1]]
        v3 = [-d[1], d[0]]

        div = np.dot(v2, v3)

        if div == 0:
            return False

        t1 = np.linalg.norm(np.cross(v2, v1)) / div
        t2 = np.dot(v1, v3) / div

        if t1 >= 0 and 0 <= t2 <= 1:
            shot = Node((o[0] + t1 * d[0], o[1] + t1 * d[1]))
            dist_obs = self.get_dist(start, shot)
            dist_seg = self.get_dist(start, end)
            if dist_obs <= dist_seg:
                return True

        return False

    # Intersection checking for a circle won't be as relevant for me but good to know

    def is_intersect_circle(self, o, d, a, r):
        d2 = np.dot(d, d)
        delta = self.delta

        if d2 == 0:
            return False

        t = np.dot([a[0] - o[0], a[1] - o[1]], d) / d2

        if 0 <= t <= 1:
            shot = Node((o[0] + t * d[0], o[1] + t * d[1]))
            if self.get_dist(shot, Node(a)) <= r + delta:
                return True

        return False

    def is_collision(self, start, end):

        # if the start or end is inside a obstacle then we know that this is a collision
        if self.is_inside_obs(start) or self.is_inside_obs(end):
                return True
            
        # get the origin of the ray, and the direction
        # start and end are points defining the ray
        o, d = self.get_ray(start, end)

        # get the vertices from the rectangle objects
        obs_vertex = self.get_obs_vertex()
            
        # vertex order is bottom left -> bottom right -> top right -> top-left
        # so check for intersection with each of the faces of the rectangle
        for (v1, v2, v3, v4) in obs_vertex:
            if self.is_intersect_rec(start, end, o, d, v1, v2):
                return True
            if self.is_intersect_rec(start, end, o, d, v2, v3):
                return True
            if self.is_intersect_rec(start, end, o, d, v3, v4):
                return True
            if self.is_intersect_rec(start, end, o, d, v4, v1):
                return True

        for (x, y, r) in self.obs_circle:
            if self.is_intersect_circle(o, d, [x, y], r):
                return True

        return False

    def is_inside_obs(self, node):
        delta = self.delta

        for (x, y, r) in self.obs_circle:
            if math.hypot(node.x - x, node.y - y) <= r + delta:
                return True

        for (x, y, w, h) in self.obs_rectangle:
            if 0 <= node.x - (x - delta) <= w + 2 * delta \
                    and 0 <= node.y - (y - delta) <= h + 2 * delta:
                return True

        for (x, y, w, h) in self.obs_boundary:
            if 0 <= node.x - (x - delta) <= w + 2 * delta \
                    and 0 <= node.y - (y - delta) <= h + 2 * delta:
                return True

        return False





    # get the ray from start to finish
    # It returns the origin of the ray and the direction
    # So an example if the ray is from (1,1) to (-1,3)
    # It will return (1,1), (-2,3)
    @staticmethod
    def get_ray(start, end):
        orig = [start.x, start.y]
        direc = [end.x - start.x, end.y - start.y]
        return orig, direc

    # I think this might be faster we shall see
    @staticmethod
    def get_dist(start, end):
        return np.linalg.norm([end.x - start.x, end.y - start.y])
        #return math.hypot(end.x - start.x, end.y - start.y)