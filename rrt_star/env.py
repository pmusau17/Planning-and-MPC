"""
Environment for path planning workspace
@author: patrick musau

inspired by huiming zhou
"""


class Env:
    def __init__(self,x_range=50,y_range=30):
        self.xr = x_range
        self.yr = y_range
        self.x_range = (0, self.xr)
        self.y_range = (0, self.yr)


    # defines visualization boundaries of work space. 
    # These are defined as rectangle patches that are displayed in matplotlib via 
    # (x,y) width, height
    def obs_boundary(self):
        obs_boundary = [
            [0, 0, 1, self.yr],
            [0, self.yr, self.xr, 1],
            [1, 0, self.xr, 1],
            [self.xr, 1, 1, self.yr]
        ]
        return obs_boundary

    # defines rectangle boundaries 
    # These are defined as rectangle patches that are displayed in matplotlib via 
    # the x,y is bottom left corner I believe
    # (x,y) width, height
    def obs_rectangle(self):
        obs_rectangle = [
            [14, 12, 8, 2],
            [18, 22, 8, 3],
            [26, 7, 2, 12],
            [32, 14, 10, 2]
        ]
        return obs_rectangle

    # defines circle boundaries 
    # These are defined as circle patches that are displayed in matplotlib via 
    # (x,y) radius  
    def obs_circle(self):
        obs_cir = [
            [7, 12, 3],
            [46, 20, 2],
            [15, 5, 2],
            [37, 7, 3],
            [37, 23, 3]
        ]

        return obs_cir