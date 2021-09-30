"""
This file configures the DAE/ODE/discrete simulator

"""

import numpy as np
import sys
from casadi import *

# Import do_mpc package:
import do_mpc

def template_simulator(model):
    # Obtain an instance of the do-mpc simulator class
    # and initiate it with the model:
    simulator = do_mpc.simulator.Simulator(model)

    params_simulator = {
    'integration_tool': 'cvodes',
    'abstol': 1e-10,
    'reltol': 1e-10,
    't_step': 1.0
    }

    # Set parameter(s):
    simulator.set_param(**params_simulator)
    
    # Optional: Set function for parameters and time-varying parameters.
    # For the simulatiom, it is necessary to define the numerical realizations of the uncertain parameters in p_num. 
    # First, we get the structure of the uncertain parameters:
    p_num = simulator.get_p_template()

    # get the uncertain parameters 
    p_num['Y_x']  = 0.4
    p_num['S_in'] = 200.0

    def p_fun(t_now):
        return p_num

    # set the user defined function above as the function for the realization of the uncertain 
    # parameters
    simulator.set_p_fun(p_fun)

    # Setup simulator:
    simulator.setup()

    return simulator