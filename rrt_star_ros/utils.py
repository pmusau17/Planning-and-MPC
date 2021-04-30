""" 
Utilities for collision checking in rrt_star
@author: patrick musau
Inspired by huiming zhou and Hongrui Zheng

Overall forom this paper: https://arxiv.org/pdf/1105.1186.pdf
"""



import math
import numpy as np
import os
import sys

# I need this file to get the occupancy grid
import plotting

# import node from rrt_star (x,y,parent)
from rrt_star import Node



self.plotting = plotting.Plotting(grid,x_start,x_goal)