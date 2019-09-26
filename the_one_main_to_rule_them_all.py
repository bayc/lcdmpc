# This is a driver to interface between co-sim and the LC-DMPC module.

# Node definitions:
# 632 - Wind
# 646 - Building
# 645 - Building
# 633 - Building
# 634 - Building
# 611 - Building
# 684 - Building
# 671 - Building
# 692 - Building
# 675 - Building
# 652 - Building
# 680 - Building

# External data needed:
#   We can hold these values constant over the prediction horizon, 
#   but if possible, it would be great to get an arbitrary amount
#   of data, based on a prediction horizon parameter.

# Buildings - these should be in the csv I sent over
#   T_oa: Outdoor air temperature
#   Q_int: Internal heat gain
#   Q_solar: Solar heat gain

# Wind - the csv file you have should be good for testing, but I think  
#        for the paper, a file with changing wind speed will be needed.
#   Ws: Wind speed
#   Wd: Wind direction

# Grid - values from openDSS
#   V: node voltages
#   P: real power

################################################
######## The beginning of pseudo-code! #########
################################################

# TODO: Do we need some sort of execute function? Or is that taken care
#       of by co-sim somehow? This may make more sense after reading
#       through the current code below.

import opendssdirect as dss
import pandas as pd
import numpy as np

def get_data(build_file_name, wind_file_name, ts, horiz_len):
    """
    Wrapper to get input data.

    Args:
        build_file_name: file name of building data
        wind_file_name: file name of wind data
        ts: current time-step
        horiz_len: length of prediction horizon

    Returns:
        list: A list of lists of the data.
    """
    build_data = get_building_data(build_file_name, ts, horiz_len)
    wind_data = get_wind_data(wind_file_name, ts, horiz_len)
    grid_data = get_grid_data()

    return [build_data, wind_data, grid_data]

def get_building_data(file_name, ts, horiz_len):
    """
    Get the building data at a certain time-step, returning 
    an array(s) of data.

    Args:
        file_name: file name of building data file
        ts: current time-step (could be done with just index?)
        horiz_len: how many time steps to return

    Returns:
        list: A list of lists containing the relevant data, 
            [T_oa, Q_int, Q_solar].
    """
    # TODO: Jen - fill in code to get values from building file.
    # General flow?
    #   read-in building data file
    #   grab the respective columns

    # return [[T_oa[ts:ts+horiz]], [Q_int[ts:ts+horiz]], [Q_solar[ts:ts+horiz]]]

def get_wind_data(file_name, ts, horiz_len):
    """
    Get the wind data at a certain time-step, returning 
    an array(s) of data.

    Args:
        file_name: file name of wind data file
        ts: current time-step (could be done with just index?)
        horiz_len: how many time steps to return

    Returns:
        list: A list of lists containing the relevant data, 
            [Ws, Wd].
    """
    # TODO: Jen - fill in code to get values from wind file.
    # General flow?
    #   read-in wind data file
    #   grab the respective columns

    # return [[Ws[ts:ts+horiz]], [Wd[ts:ts+horiz]]]

def get_grid_data():
    """
    Get the power and voltages from the nodes in openDSS to use as 
    linearization points for the voltage constraints. I think just
    grabbing them wholesale works, and we can parse them elsewhere.

    Returns:
        list: A list of lists with the power and voltages,
            [P, V].
    """

    # TODO: Jen - fill in code to get values from openDSS.

    # return [[P], [V]]

def set_truth_inputs(ms_dot, T_sa, yaw, Ct, node_powers):
    """
    Wrapper for the other setting functions for truth models and openDSS.
    
    Args:
        ms_dot: mass air flow rate driven by the AHU fan
        T_sa: supply air temperature (Celsius)
        yaw: array of yaw values for turbines in wind farm
        Ct: array of Ct values for turbines in wind farm
        node_powers: A dictionary? Containing pairs of node IDs and powers?
    """
    # TODO: Set this up to loop through buildings
    build_powers = set_building_truth(ms_dot, T_sa)
    wind_powers = set_wind_truth(yaw, Ct)
    set_grid_powers([build_powers, wind_powers])

def set_building_truth(ms_dot, T_sa):
    """
    Assign control actions to truth model. It will also need inputs 
    from the data files.

    Args:
        ms_dot: mass air flow rate driven by the AHU fan
        T_sa: supply air temperature (Celsius)
    """

    # TODO: Jen - set control actions in truth model; also figure out 
    #             how to set other model input from data file:
    #             T_oa, Q_int, and Q_solar.

def set_wind_truth(yaw, Ct):
    """
    Assign control actions to truth model. It will also need inputs 
    from the data files.

    Args:
        yaw: array of yaw values for turbines in wind farm
        Ct: array of Ct values for turbines in wind farm
    """

    # TODO: Jen - set control actions in truth model; also figure out 
    #             how to set other model input from data file:
    #             Ws and Wd.

def set_grid_powers(node_powers):
    """
    Set the node powers in openDSS so it cn calculate the voltages.
    
    Args:
        node_powers: A dictionary? Containing pairs of node IDs and powers?
    """

    # TODO: Jen - write code to set node powers in openDSS.


# TODO: Chris - make this file work with current LC-DMPC module. The code 
#               below is just to show the general flow of things. I still
#               need to figure out the best way to interface between here
#               and there. 

time = 20
dt = 5
commuincation_iterations = 1

for i in range(int(time/dt)):

    # inputs = get_data()

    # TODO: Chris - write parser for input data
    # TODO: Chris - add Rohit's filter update for building control model

    # opt.relinearize_subsystem_models(inputs)  # Relinearize to current point

    for j in range(commuincation_iterations):
        # opt.communicate()         # Exchange info between susbsystems

        # opt.optimize_all()        # Solve optimization for each subsys

        # opt.convex_sum_cont()     # Convex combination of control action

        # opt.update_downstream_outputs()   # Update Z vectors to be passed

        # opt.calculate_sensitivities()     # Calculate gamma's to be passed

        print('==============================')
        print('communication iteration: ', j)
        print('==============================')

    # opt.update_states()

    # opt.update_subsystem_outputs()

    # opt.update_inputs_for_linearization()

    # TODO: Chris - write function to provide setpoints for truth models
    # TODO: Jen - write function to apply setpoints to truth models; this 
    #             includes building and wind model inputs as well as
    #             power injections for openDSS.

    print('+++++++++++++++++++++++++++++')
    print('time iteration: ', i)
    print('+++++++++++++++++++++++++++++')