"""

This file defines the dynamic model.

The considered model of the batch bioreactor is continuous and has 4 states and 1 control input, which are depicted below:


"""

import numpy as np
import sys

#CasADi is an open-source software tool for numerical optimization in general and optimal control (i.e. optimization involving differential equations) in particular. 
from casadi import *


# Import do_mpc package:
import do_mpc

def template_model():
    # Obtain an instance of the do-mpc model class
    # and select time discretization:

    model_type = 'continuous' # either 'discrete' or 'continuous'
    model = do_mpc.model.Model(model_type)

    # States struct (optimization variables):
    X_s = model.set_variable('_x',  'X_s') # X_s the concentration of the substrate
    S_s = model.set_variable('_x',  'S_s')
    P_s = model.set_variable('_x',  'P_s')
    V_s = model.set_variable('_x',  'V_s')

    # The control input is the feed flow rate 𝑢inp of 𝑆s:
    inp = model.set_variable('_u',  'inp')

    # Certain parameters
    mu_m  = 0.02
    K_m   = 0.05
    K_i   = 5.0
    v_par = 0.004
    Y_p   = 1.2

    # Uncertain parameters:
    Y_x  = model.set_variable('_p',  'Y_x')
    S_in = model.set_variable('_p', 'S_in')

    # Auxiliary term
    mu_S = mu_m*S_s/(K_m+S_s+(S_s**2/K_i))

    # Directly from the Differential equations
    model.set_rhs('X_s', mu_S*X_s - inp/V_s*X_s)
    model.set_rhs('S_s', -mu_S*X_s/Y_x - v_par*X_s/Y_p + inp/V_s*(S_in-S_s))
    model.set_rhs('P_s', v_par*X_s - inp/V_s*P_s)
    model.set_rhs('V_s', inp)

    # Build the model
    model.setup()

    return model 