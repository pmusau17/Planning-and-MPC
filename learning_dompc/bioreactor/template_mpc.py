""" 

This file configures the MPC controller

"""

import numpy as np
import sys
from casadi import *

# Import do_mpc package:
import do_mpc

def template_mpc(model):
    # Obtain an instance of the do-mpc MPC class
    # and initiate it with the model:

    mpc = do_mpc.controller.MPC(model)

    # We choose the prediction horizon n_horizon, set the robust horizon n_robust to 3. 
    # The time step t_step is set to one second and parameters of the applied 
    # discretization scheme orthogonal collocation are as seen below:

    setup_mpc = {
    'n_horizon': 20,
    'n_robust': 1,
    'open_loop': 0,
    't_step': 1.0,
    'state_discretization': 'collocation',
    'collocation_type': 'radau',
    'collocation_deg': 2,
    'collocation_ni': 2,
    'store_full_solution': True,
    # Use MA27 linear solver in ipopt for faster calculations:
    #'nlpsol_opts': {'ipopt.linear_solver': 'MA27'}
    }

    mpc.set_param(**setup_mpc)

    # The batch bioreactor is used to produce penicillin. Hence, the objective of the controller is to maximize the concentration of the product 𝑃s. 
    # Additionally, we add a penalty on input changes, to obtain a smooth control performance.
    # It's a minimization problem so the penalty is negative.
    mterm = -model.x['P_s'] # stage cost
    lterm = -model.x['P_s'] # terminal cost

    mpc.set_objective(mterm=mterm, lterm=lterm)
    # penalty on input changes
    mpc.set_rterm(inp=1.0) 

    # In the next step, the constraints of the control problem are set. 
    # In this case, there are only upper and lower bounds for each state and the input:

    # lower bounds of the states
    mpc.bounds['lower', '_x', 'X_s'] = 0.0
    mpc.bounds['lower', '_x', 'S_s'] = -0.01
    mpc.bounds['lower', '_x', 'P_s'] = 0.0
    mpc.bounds['lower', '_x', 'V_s'] = 0.0

    # upper bounds of the states
    mpc.bounds['upper', '_x','X_s'] = 3.7
    mpc.bounds['upper', '_x','P_s'] = 3.0

    # upper and lower bounds of the control input
    mpc.bounds['lower','_u','inp'] = 0.0
    mpc.bounds['upper','_u','inp'] = 0.2


    Y_x_values = np.array([0.5, 0.4, 0.3])
    S_in_values = np.array([200.0, 220.0, 180.0])

    mpc.set_uncertainty_values(Y_x = Y_x_values, S_in = S_in_values)

    mpc.setup()

    return mpc