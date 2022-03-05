import numpy as np
import math
from casadi import *
from casadi.tools import *
import sys
import pdb
import do_mpc

def mpc_model(sg, symvar_type='SX'):

    # Selecting tyme discretization choice
    model_type = 'continuous'

    # Initializing the model of the spacecraft
    model = do_mpc.model.Model(model_type)

    # The state variables:
    X = model.set_variable('_x',  'X')  # x-axis position
    Y = model.set_variable('_x',  'Y')  # y-axis position
    psi = model.set_variable('_x',  'psi')  # orientation angle
    v_X = model.set_variable('_x',  'v_X')  # horizontal velocity
    v_Y = model.set_variable('_x',  'v_Y')  # vertical velocity
    d_psi = model.set_variable('_x',  'd_psi')  # Angular rate
        
    # Set points to track
    X_set_point = model.set_variable(var_type = '_tvp', var_name = 'X_set_point')
    Y_set_point = model.set_variable(var_type = '_tvp', var_name = 'Y_set_point')
    # Set points to avoid
    X_ostacle = model.set_variable(var_type = '_tvp', var_name = 'X_obstacle')
    Y_obstacle = model.set_variable(var_type = '_tvp', var_name = 'Y_obstacle')
    # Set discounted factors
    discounted_factor = model.set_variable(var_type = '_tvp', var_name = 'discounted_factor')
    pose_discounted_factor = model.set_variable(var_type = '_tvp', var_name = 'pose_discounted_factor')

    # Control input to be optimized:
    u_acc_lx = model.set_variable('_u',  'u_acc_lx')        #acceleration left propeller 
    u_acc_rx = model.set_variable('_u',  'u_acc_rx')        #acceleration right propeller

    # Differential equations (setting up the right hand side of the equations)
    d_X = v_X * cos(psi) - v_Y * sin(psi)
    model.set_rhs('X', d_X)
    d_Y = v_X * sin(psi) + v_Y * cos(psi)
    model.set_rhs('Y', d_Y)
    model.set_rhs('psi', d_psi)
    dd_X = (u_acc_lx + u_acc_rx) + v_Y * d_psi
    model.set_rhs('v_X', dd_X)
    dd_Y = - v_X * d_psi
    model.set_rhs('v_Y', dd_Y)
    dd_psi = sg.w_half * sg.m / sg.Iz * (u_acc_rx - u_acc_lx)
    model.set_rhs('d_psi',  dd_psi) 

    # Build the model
    model.setup()

    return model

def mpc_tracking(model, sg, sp, boundaries, smoothed_path, n_horizon, avoidance = False, safety_distance = 7, extra_distance = 0.5, dyn_obstacle_poses = []):

    mpc = do_mpc.controller.MPC(model)

    setup_mpc = {
        'n_horizon': n_horizon,
        'n_robust': 0,
        'open_loop': 0,
        't_step': 0.1,
        'state_discretization': 'collocation',
        'collocation_type': 'radau',
        'collocation_deg': 1,
        'collocation_ni': 1,
        'store_full_solution': True,
        # Use MA27 linear solver in ipopt for faster calculations:
        #'nlpsol_opts': {'ipopt.linear_solver': 'mumps'}
    }

    mpc.set_param(**setup_mpc)

    # set weight of the costs
    xy_weight = 15 # weight for position error cost
    v_Y_weight = 10 # weight for lateral movement cost
    d_psi_weight = 15 # weight for rotation rate cost
    avoidance_weight = 50 # weight for avoidance cost

    # MPC cost in normal condition
    if not avoidance:
        mterm = model.tvp['discounted_factor']*( xy_weight*((model.x['X'] - model.tvp['X_set_point'])**2) + xy_weight*((model.x['Y'] - model.tvp['Y_set_point'])**2) +
                v_Y_weight*(model.x['v_Y'])**2 + d_psi_weight*(model.x['d_psi'])**2)
        lterm = model.tvp['discounted_factor']*( xy_weight*((model.x['X'] - model.tvp['X_set_point'])**2) + xy_weight*((model.x['Y'] - model.tvp['Y_set_point'])**2) +
                v_Y_weight*(model.x['v_Y'])**2 + d_psi_weight*(model.x['d_psi'])**2)

    # MPC cost near a dynamic obstacle
    else:                   
        mterm = model.tvp['discounted_factor']*( avoidance_weight**(-(model.tvp['X_obstacle']-model.x['X'])**2 - (model.tvp['Y_obstacle']-model.x['Y'])**2 + (safety_distance + extra_distance)**2)+
                model.tvp['pose_discounted_factor']*(xy_weight*((model.x['X'] - model.tvp['X_set_point'])**2) + xy_weight*((model.x['Y'] - model.tvp['Y_set_point'])**2)) +
                v_Y_weight*(model.x['v_Y'])**2 + d_psi_weight*(model.x['d_psi'])**2)
        lterm = model.tvp['discounted_factor']*( avoidance_weight**(-(model.tvp['X_obstacle']-model.x['X'])**2 - (model.tvp['Y_obstacle']-model.x['Y'])**2 + (safety_distance + extra_distance)**2)+
                model.tvp['pose_discounted_factor']*(xy_weight*((model.x['X'] - model.tvp['X_set_point'])**2) + xy_weight*((model.x['Y'] - model.tvp['Y_set_point'])**2)) +
                v_Y_weight*(model.x['v_Y'])**2 + d_psi_weight*(model.x['d_psi'])**2)

    # Set state costs
    mpc.set_objective(mterm=mterm, lterm=lterm)

    # Set input costs
    mpc.set_rterm(
    u_acc_lx=0.1,
    u_acc_rx=0.1
    )

    tvp_template = mpc.get_tvp_template()

    def tvp_fun(t_now):
        spread_pts = 1
        
        if len(smoothed_path) < spread_pts*(setup_mpc['n_horizon']):
            n_horizon_tvp = int(floor(len(smoothed_path)/spread_pts) - 2)
        else:
            n_horizon_tvp = setup_mpc['n_horizon']

        for k in range(n_horizon_tvp + 1):
                tvp_template['_tvp', k, 'X_set_point'] = smoothed_path[spread_pts*k][0]
                tvp_template['_tvp', k, 'Y_set_point'] = smoothed_path[spread_pts*k][1]
                tvp_template['_tvp', k, 'discounted_factor'] = 0.95**k
                
                if avoidance:
                    tvp_template['_tvp', k, 'pose_discounted_factor'] = 0.97**k
                    tvp_template['_tvp', k, 'X_obstacle'] = dyn_obstacle_poses[k+1][0]
                    tvp_template['_tvp', k, 'Y_obstacle'] = dyn_obstacle_poses[k+1][1]

        return tvp_template  

    mpc.set_tvp_fun(tvp_fun)

    #constraints on the optimization variables

    #lower bounds on the state variables		
    mpc.bounds['lower', '_x', 'X'] = boundaries[0] + max(sg.lf+sg.lr, 2*sg.w_half)
    mpc.bounds['lower', '_x', 'Y'] = boundaries[1] + max(sg.lf+sg.lr, 2*sg.w_half)
    mpc.bounds['lower', '_x', 'v_X'] = sp.vx_limits[0]/3
    mpc.bounds['lower', '_x', 'd_psi'] = sp.dpsi_limits[0]

    #upper bounds on the state variables
    mpc.bounds['upper', '_x', 'X'] = boundaries[2] - max(sg.lf+sg.lr, 2*sg.w_half)
    mpc.bounds['upper', '_x', 'Y'] = boundaries[3] - max(sg.lf+sg.lr, 2*sg.w_half)
    mpc.bounds['upper', '_x', 'v_X'] = sp.vx_limits[1]
    mpc.bounds['upper', '_x', 'd_psi'] = sp.dpsi_limits[1]

    #lower bounds on the inputs
    mpc.bounds['lower','_u','u_acc_lx'] = sp.acc_limits[0]
    mpc.bounds['lower','_u','u_acc_rx'] = sp.acc_limits[0]

    #upper bounds on the inputs
    mpc.bounds['upper','_u','u_acc_lx'] = sp.acc_limits[1]
    mpc.bounds['upper','_u','u_acc_rx'] = sp.acc_limits[1]
    
    mpc.setup()

    return mpc

   




