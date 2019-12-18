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
# JK: AES.run() should execute cosim at least.

import pandas as pd
import numpy as np
import aes

# initial parameters

# start and stop times
starttime = "2019-02-15 00:00:00"
stoptime = "2019-02-15 00:10:00"

# initialize AES
# TODO: maybe put all the other inputs into a json.  Currently all inputs exist in aes.py
scenario = aes.AES(starttime,stoptime)

# run scenario
scenario.run()

# # TODO: Chris - make this file work with current LC-DMPC module. The code
# #               below is just to show the general flow of things. I still
# #               need to figure out the best way to interface between here
# #               and there.
#
# time = 20
# dt = 5
# commuincation_iterations = 1
#
# for i in range(int(time/dt)):
#
#     # inputs = get_data()
#
#     # TODO: Chris - write parser for input data
#     # TODO: Chris - add Rohit's filter update for building control model
#
#     # opt.relinearize_subsystem_models(inputs)  # Relinearize to current point
#
#     for j in range(commuincation_iterations):
#         # opt.communicate()         # Exchange info between susbsystems
#
#         # opt.optimize_all()        # Solve optimization for each subsys
#
#         # opt.convex_sum_cont()     # Convex combination of control action
#
#         # opt.update_downstream_outputs()   # Update Z vectors to be passed
#
#         # opt.calculate_sensitivities()     # Calculate gamma's to be passed
#
#         print('==============================')
#         print('communication iteration: ', j)
#         print('==============================')
#
#     # opt.update_states()
#
#     # opt.update_subsystem_outputs()
#
#     # opt.update_inputs_for_linearization()
#
#     # TODO: Chris - write function to provide setpoints for truth models
#     # TODO: Jen - write function to apply setpoints to truth models; this
#     #             includes building and wind model inputs as well as
#     #             power injections for openDSS.
#
#     print('+++++++++++++++++++++++++++++')
#     print('time iteration: ', i)
#     print('+++++++++++++++++++++++++++++')