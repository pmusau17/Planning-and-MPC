""" Plotting Tools for Sampling-based algorithms
@author: Patrick Musau

Inspiried by Huming Zhou
"""
import env
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import os
import sys

class Plotting:

    """
    x_start: 2D Point
    y_start: 2D Point
    """


    def __init__(self, x_start, x_goal,x_width=50,y_width=30):
        self.xI, self.xG = x_start, x_goal
        # create object that defines the planning workspace
        self.env = env.Env(x_width,y_width)
        self.obs_bound = self.env.obs_boundary()
        self.obs_circle = self.env.obs_circle()
        self.obs_rectangle = self.env.obs_rectangle()
        self.fig, self.ax = plt.subplots()
    
    def animation(self,nodelist,path,name, animation=False):
        # plot the grid
        self.plot_grid(name)
        # plot visited nodes
        self.plot_visited(nodelist, animation)
        # plot the final path 
        self.plot_path(path)

    
    def animation_connect(self, V1, V2, path, name):
        self.plot_grid(name)
        self.plot_visited_connect(V1, V2)
        self.plot_path(path)

    
    # plot the 2d grid
    def plot_grid(self, name):
        

        for (ox, oy, w, h) in self.obs_bound:
            self.ax.add_patch(
                patches.Rectangle(
                    (ox, oy), w, h,
                    edgecolor='black',
                    facecolor='black',
                    fill=True
                )
            )

        for (ox, oy, w, h) in self.obs_rectangle:
            self.ax.add_patch(
                patches.Rectangle(
                    (ox, oy), w, h,
                    edgecolor='black',
                    facecolor='gray',
                    fill=True
                )
            )

        for (ox, oy, r) in self.obs_circle:
            self.ax.add_patch(
                patches.Circle(
                    (ox, oy), r,
                    edgecolor='black',
                    facecolor='gray',
                    fill=True
                )
            )

        # plot the initial point and goal
        self.ax.plot(self.xI[0], self.xI[1], "bs", linewidth=3)
        self.ax.plot(self.xG[0], self.xG[1], "gs", linewidth=3)
        plt.title(name)

        # set equal aspect ratio for figures
        plt.axis("equal")
        plt.show(block=False)


    # Go through the list of nodes and connect each node to it's parent 
    # Pausing if you want to animate
    #@staticmethod
    def plot_visited(self,nodelist, animation):
        if animation:
            count = 0
            for node in nodelist:
                count += 1
                if node.parent:
                    self.ax.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")
                    plt.gcf().canvas.mpl_connect('key_release_event',
                                                 lambda event:
                                                 [exit(0) if event.key == 'escape' else None])
                    if count % 10 == 0:
                        plt.pause(0.001)
        else:
            for node in nodelist:
                if node.parent:
                    plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")

    

    # Not exactly sure yet but we will get there
    # Looks like plotting two sets of vertices not sure how this works
    @staticmethod
    def plot_visited_connect(V1, V2):
        len1, len2 = len(V1), len(V2)

        for k in range(max(len1, len2)):
            if k < len1:
                if V1[k].parent:
                    plt.plot([V1[k].x, V1[k].parent.x], [V1[k].y, V1[k].parent.y], "-g")
            if k < len2:
                if V2[k].parent:
                    plt.plot([V2[k].x, V2[k].parent.x], [V2[k].y, V2[k].parent.y], "-g")

            plt.gcf().canvas.mpl_connect('key_release_event',
                                         lambda event: [exit(0) if event.key == 'escape' else None])

            if k % 2 == 0:
                plt.pause(0.001)

        plt.pause(0.01)



    # plotting a path via list comprehensions
    #@staticmethod
    def plot_path(self,path):
        if len(path) != 0:
            self.ax.plot([x[0] for x in path], [x[1] for x in path], '-r', linewidth=2)
            plt.pause(0.01)
        plt.show(block=False)
