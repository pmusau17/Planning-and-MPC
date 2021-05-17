""" Plotting Tools for Sampling-based algorithms
@author: Patrick Musau

Inspiried by Huming Zhou
"""
#import env
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import os
import sys

class Plotting:

    """
    x_start: 2D Point
    y_start: 2D Point

    free (0), 
    occupied (100), 
    and unknown (-1)
    """


    def __init__(self, grid_file,x_start,x_goal):

        # create the figure
        self.fig, self.ax = plt.subplots(figsize=(20, 15))

        # the origin and res come from the map.yaml file
        self.origin= (-100,-100)
        self.res = 0.04
        self.grid=np.load(grid_file).astype(int)
        item = (np.where(self.grid>0))
        item2 = (np.where(self.grid==0))
        # just plot the boundaries
        self.x_obs = (item[0] * self.res) + self.origin[0]
        self.y_obs = (item[1] * self.res) + self.origin[1]
        self.x_free = (item2[0] * self.res) + self.origin[0]
        self.y_free = (item2[1] * self.res) + self.origin[1]
        self.count =0
    
        
        # get the start and goal
        self.xI, self.xG = x_start, x_goal
        # create object that defines the planning workspace
    
    def animation(self,nodelist,path,name, animation=False,block=False):
        # plot the grid
        #self.ax.clear()
        self.plot_grid(name)
        # plot visited nodes
        self.plot_visited(nodelist, animation)
        # plot the final path 
        self.plot_path(path,block=block)

    """
        I'm a visual learner so this method just shows me how points are sampled
    """
    def random_sample(self):
        x = np.random.choice(self.x_free)
        y = np.random.choice(self.y_free)
        x_index = int(round((x - self.origin[0])/(self.res)))
        y_index = int(round((y - self.origin[1])/(self.res)))
        print(x,y,x_index,y_index)
        print("Sanity Check")
        print((x_index*self.res)+self.origin[0],(y_index*self.res)+self.origin[1],)
        print(x_index,y_index,self.grid[x_index][y_index])
        self.ax.plot([x], [y], "ks", linewidth=3,label="random_point")
        plt.legend()
        plt.show(block=True)


    
    # plot the 2d grid
    def plot_grid(self, name,block=False):
        


        self.ax.plot(self.x_obs,self.y_obs,'k.',label="boundaries")
        self.ax.plot(self.x_free,self.y_free,'y.',label="freespace")

        self.ax.plot(self.xI[0], self.xI[1], "bs", linewidth=3,label="init")
        self.ax.plot(self.xG[0], self.xG[1], "ms", linewidth=3,label="goal")

        # set equal aspect ratio for figures
        plt.xlabel("x(m)")
        plt.ylabel("y(m)")
        plt.title(name)
        plt.axis("equal")
        
        plt.show(block=block)
        if(self.count<1):
            plt.legend()
        self.count+=1


    # Go through the list of nodes and connect each node to it's parent 
    # Pausing if you want to animate
    #@staticmethod
    def plot_visited(self,nodelist, animation):
        if animation:
            count = 0
            for node in nodelist:
                count += 1
                if node.parent:
                    self.ax.plot([node.parent.x, node.x], [node.parent.y, node.y], "-.g")
                    plt.gcf().canvas.mpl_connect('key_release_event',
                                                 lambda event:
                                                 [exit(0) if event.key == 'escape' else None])
                    if count % 10 == 0:
                        plt.pause(0.001)
        else:
            for node in nodelist:
                if node.parent:
                    plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-.g")

    # plotting a path via list comprehensions
    #@staticmethod
    def plot_path(self,path,block=False):
        if len(path) != 0:
            self.ax.plot([x[0] for x in path], [x[1] for x in path], '-r', linewidth=2)
            plt.pause(0.01)
        plt.show(block=block)


if __name__ == "__main__":
    x_start = (0,0)
    x_goal = (1.422220,1.244794)
    grid = 'porto_grid.npy'
    pl = Plotting(grid,x_start,x_goal)
    pl.plot_grid("Porto Occupancy Grid",block=False)
    pl.random_sample()
