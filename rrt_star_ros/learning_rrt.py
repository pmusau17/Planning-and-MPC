import numpy as np 
import seaborn as sns
import matplotlib.pyplot as plt
#sns.set_theme(style="white")

# Node for RRT
# np uniform generator is np.random.uniform(low,high,size)
class Node:
    def __init__(self,x=None,y=None,cost=0.0,parent_index=None):
        self.x = x
        self.y = y 
        self.cost = 0.0
        self.parent_index = parent_index





if __name__=="__main__":
    grid=np.load('porto_grid.npy')
    print(grid)
    ##grid[2210:2270]
    #print(grid.shape)
    #plt.imshow(grid)
    #plt.xlim(2210,2857)
    #plt.ylim(2400,2700)
    #plt.show()
