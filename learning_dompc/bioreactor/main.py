"""
This file obtains all the configured modules and runs the loop
"""


import numpy as np
import sys
from casadi import *

# Import do_mpc package:
import do_mpc


# import definitions from other files
# in hindsight I don't know why they think this 
# is a good structure

from template_model import template_model
from template_mpc import template_mpc
from template_estimator import template
from template_simulator import template_simulator


# Preparing the visualzation in matplotlib
import matplotlib.pyplot as plt 
plt.ion()
from matplotlib import rcParams
import matplotlib as mpl

# si tu veux, vous pouvez utiliser de l'animation
from matplotlib.animation import FuncAnimation, FFMpegWriter, ImageMagickWriter

# set params for do-mpc
rcParams['text.usetex'] = True
rcParams['text.latex.preamble'] = [r'\usepackage{amsmath}',r'\usepackage{siunitx}']
rcParams['axes.grid'] = True
rcParams['lines.linewidth'] = 2.0
rcParams['axes.labelsize'] = 'xx-large'
rcParams['xtick.labelsize'] = 'xx-large'
rcParams['ytick.labelsize'] = 'xx-large'


if __name__ == "__main__":

    # define the model 
    model = template_model()

    # set flag to see whether you want a static plot or an animation 
    static = False

    # define the mpc controller 
    mpc = template_mpc(model)

    # define the simulator 
    simulator = template_simulator(model)

    # define the estimator 
    estimator = template(model)

    # closed loop simulation

    X_s_0 = 1.0 # Concentration biomass   [mol/l]
    S_s_0 = 0.5 # Concentration substrate [mol/l]
    P_s_0 = 0.0 # Concentration product   [mol/l]
    V_s_0 = 120.0

    x0 = np.array([X_s_0,S_s_0,P_s_0,V_s_0])

    # set for controller, simulator, and estimator 
    mpc.x0 = x0 
    simulator.x0 = x0
    estimator.x0 = x0
    mpc.set_initial_guess()

    # visualization 
    mpc_graphics = do_mpc.graphics.Graphics(mpc.data)
    sim_graphics = do_mpc.graphics.Graphics(simulator.data)

    fig, ax = plt.subplots(5, sharex=True, figsize=(16,9))
    fig.align_ylabels()

    for g in [sim_graphics,mpc_graphics]:
        # Plot the state on axis 1 to 4:
        g.add_line(var_type='_x', var_name='X_s', axis=ax[0], color='#1f77b4')
        g.add_line(var_type='_x', var_name='S_s', axis=ax[1], color='#1f77b4')
        g.add_line(var_type='_x', var_name='P_s', axis=ax[2], color='#1f77b4')
        g.add_line(var_type='_x', var_name='V_s', axis=ax[3], color='#1f77b4')

        # Plot the control input on axis 5:
        g.add_line(var_type='_u', var_name='inp', axis=ax[4], color='#1f77b4')


    ax[0].set_ylabel(r'$X_s~[\si[per-mode=fraction]{\mole\per\litre}]$')
    ax[1].set_ylabel(r'$S_s~[\si[per-mode=fraction]{\mole\per\litre}]$')
    ax[2].set_ylabel(r'$P_s~[\si[per-mode=fraction]{\mole\per\litre}]$')
    ax[3].set_ylabel(r'$V_s~[\si[per-mode=fraction]{\mole\per\litre}]$')
    ax[4].set_ylabel(r'$u_{\text{inp}}~[\si[per-mode=fraction]{\cubic\metre\per\minute}]$')
    ax[4].set_xlabel(r'$t~[\si[per-mode=fraction]{\minute}]$')
    

    # do the closed loop simulation 
    n_steps = 10
    for k in range(n_steps):
        u0 = mpc.make_step(x0)
        y_next = simulator.make_step(u0)
        x0 = estimator.make_step(y_next)

    if(static):
        sim_graphics.plot_results()
        sim_graphics.reset_axes()
        mpc_graphics.plot_results()
        plt.show(block=True)
    else:
        # The function describing the gif:
        def update(t_ind):
            sim_graphics.plot_results(t_ind)
            mpc_graphics.plot_predictions(t_ind)
            mpc_graphics.reset_axes()

        anim = FuncAnimation(fig, update, frames=n_steps, repeat=False)
        gif_writer = ImageMagickWriter(fps=10)
        anim.save('anim_batch_reactor_final.gif', writer=gif_writer)
    


