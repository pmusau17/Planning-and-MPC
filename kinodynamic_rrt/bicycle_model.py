"""
Bicycle Model Class
author Patrick Musau
"""

import numpy as np
import copy
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
        T:   simulation_time
        x:   xs[0] pos (m)
        y:   xs[1] pos (m)
        yaw: xs[2] radians
        v:   xs[3] velocity m/s
    """
    def __init__(self,xs=[0,0,0,0]):
        self.x = xs 
        self.beta=0
    """
        euler step 
        x = x + (x' * dt)
    """ 

    """ 
        Use euler simulation to simulate the trajectory forward
        returns 4d array of simulation
        U = control input [throttle, steering angle]
        T = Simulation Horizion (s)
    """
    def simulate_forward(self,u,T,dt=0.01):
        steps = int(T / dt)
        trace = [self.x] 
        for i in range(steps+1):
            trace.append(self.update(u,dt))
        return np.asarray(trace)


    """
        Simulate trajectories with an underlying speed and a set of steering angles

        throttle: speed we want the car to move along the trajectory
        num_primitives: how many unique trajectories that are considered as motion primitives
        steer_min, steer_max = maximum turning action considered for the motion primitives 
        T : predictionhorizon  
        
    """
    def simulate_motion_primitives(self,throttle=0.3,num_primitives=5,steer_min=-0.61,steer_max=0.61,T=0.5,dt=0.01,plot=False):
        steering_angles = np.linspace(steer_min,steer_max,num_primitives)
        traces = []
        start = self.x[:]
        for i in range(num_primitives):
            u = [throttle,steering_angles[i]]
            tr = self.simulate_forward(u,T,dt)
            traces.append(tr)
            self.x = start[:]
        return traces

            


    def update(self, uc,dt=0.01):
        u         = uc[0]
        delta     = uc[1]
        self.x[0]   = self.x[0] + (self.x[3] * math.cos(self.x[2] + self.beta)) * dt
        self.x[1]   = self.x[1] + (self.x[3] * math.sin(self.x[2] + self.beta)) * dt
        self.x[3]   = self.x[3] + (-ca * self.x[3] + ca*cm*(u - ch)) * dt
        self.x[2] = self.x[2] + (self.x[3] * (math.cos(beta)/(lf+lr)) * math.tan(delta)) * dt
        return [self.x[0],self.x[1],self.x[2],self.x[3]]

    # This might help let's see
    def pi_2_pi(self,angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

# Testing only
if __name__ == '__main__':
    import matplotlib.pyplot as plt
    
    # two second simulation 
    T = 1
    u = [1.0,0.266]
    u2 = [1.0,-0.266]
    u3 = [1.0,-0.61]
    u4 = [1.0,-0.61]
    dt = 0.05

    s = State()
    num_primitives = 8
    traces = s.simulate_motion_primitives(T=T,num_primitives=num_primitives,throttle=1.0,dt=dt)

    for i in range(num_primitives):
        tr1 = traces[i]
        x = tr1[:,0]
        y = tr1[:,1]
        plt.plot(x, y,'.')

    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.axis("equal")
    plt.grid(True)
    plt.show()

    # need to see if I set the speed to zero if something bad happens lol
    s = State()
    tr = s.simulate_forward(u,T,dt)
    x = tr1[:,0]
    y = tr1[:,1]
    plt.plot(x, y,'.')

    # from the previous solution 
    s = State(tr1[-1])
    tr1 = s.simulate_forward(u,T,dt)
    x = tr1[:,0]
    y = tr1[:,1]
    plt.plot(x, y,'.')
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.axis("equal")
    plt.grid(True)
    plt.show()






    
