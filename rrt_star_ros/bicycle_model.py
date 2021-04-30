"""
Bicycle Model Class
author Patrick Musau
"""

import numpy as np
import math

""" """
### Parameters for Bicycle Model Obtained through system Identification 
ca = 1.9569
cm = 0.0342
ch = -37.1967

# distance from back to rear wheels
lf = 0.225
lr = 0.225
beta  = 0

class State:

    """
        dt: time step used for euler simulation
        T: simulation_time
        x: pos (m)
        y: pos (m)
        yaw: radians
        v: velocity m/s
    """
    def __init__(self,x=0,y=0,yaw=0,v=0):
        self.x = x 
        self.y = y
        self.v = v
        self.yaw = yaw
"""
        euler step 
        x = x + (x' * dt)
""" 


def update(state, u,delta):
        state.x   = state.x + (state.v * math.cos(state.yaw + beta)) * dt
        state.y   = state.y + (state.v * math.sin(state.yaw + beta)) * dt
        state.v   = state.v + (-ca * state.v + ca*cm*(u - ch)) * dt
        state.yaw = state.yaw + (state.v * (math.cos(beta)/(lf+lr)) * math.tan(delta)) * dt
        return state

# This might help let's see
def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

# Testing only
if __name__ == '__main__':
    import matplotlib.pyplot as plt
    
    # two second simulation 
    T = 200
    dt = 0.01
    u = [1.0]*T
    # left
    delta = [0.2666] * T

    # right
    delta2 = [-0.2666] * T

    # random
    delta3 = np.random.uniform(-0.61,0.61,T)
    print(delta3)
    state = State()
    state2 = State()
    state3 = State()
    x = []
    x2 = []
    x3 = []
    y = []
    y2 = []
    y3 = []
    yaw = []
    yaw2 = []
    yaw3 = []
    v = []
    v2 = []
    v3 = []

    # Do euler simulation 
    for (ui,di,d2,d3) in zip(u,delta,delta2,delta3):
        state = update(state, ui, di)
        state2 = update(state2, ui, d2)
        state3 = update(state3, ui, d3)
        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)

        x2.append(state2.x)
        y2.append(state2.y)
        yaw2.append(state2.yaw)
        v2.append(state2.v)

        x3.append(state3.x)
        y3.append(state3.y)
        yaw3.append(state3.yaw)
        v3.append(state3.v)

    plt.plot(x, y,'b.')
    plt.plot(x2, y2,'r.')
    plt.plot(x3, y3,'g.')
    plt.axis("equal")
    plt.grid(True)
    plt.show()




    
