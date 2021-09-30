"""
This function configures the estimator (MHE / EKF / state-feedback)

"""

import numpy as np
import sys
from casadi import *

# Import do_mpc package:
import do_mpc

def template(model):
    
    # In this example we assume that all the states can be directly 
    # measured (state-feedback)

    estimator = do_mpc.estimator.StateFeedback(model)
  
    

    return estimator