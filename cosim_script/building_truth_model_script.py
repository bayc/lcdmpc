import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import bldg_sim_mdl

# Disturbance data for the simulation
disturbance_file = 'ROM_simulation_data.csv'
disturbance_data = pd.read_csv(disturbance_file)

# Simulation time information
start_time = 750    # Starting time of the simulation, in minutes
run_time = 240      # How long the simulation should run for, in minutes
dt = 5              # Time-step of the simulation, in minutes

# Initial model parameter values
ms_dot = 8.0    # Mass flow rate of air through AHU
T_sa = 12.8     # Supply air temperature from AHU
T_z = 22.794    # Zone air temperature
T_e = 20.0      # Zone effective wall temperature

# Simulated control values to be applied to model
ms_dot = ms_dot*np.ones(int(run_time/dt))    # Mass flow rate of air through AHU
T_sa = T_sa*np.ones(int(run_time/dt))     # Supply air temperature from AHU
Q_hvac = np.zeros(len(ms_dot))          # Place holder (value is used for constraints)

# Instantiate the building truth model
# NOTE: The building truth model is expecting a time-step in hours, so must convert
bldg1_truth = bldg_sim_mdl.bldg_sim_mdl(dt/12, ms_dot, T_sa, T_z, T_e, start_time)

# Placeholders for the output from the building truth model
T_bldg = []
P_bldg = []

# Simulation loop
for i in range(0, int(run_time/dt)):
    # current_time is used to grab the correct disturbance data
    current_time = start_time + i*dt

    # These are the control values that are applied to the truth model
    inputs = [Q_hvac[i], ms_dot[i], T_sa[i]]

    # These are the disturbance values read from the disturbance file
    disturbances = bldg1_truth.get_forecast(current_time, disturbance_data)

    # These are the outputs from the building truth model
    outputs = bldg1_truth.simulate(current_time, inputs, disturbances)
    
    # Parsing the building truth model outputs for easier handling
    T_bldg.append(outputs[0])
    P_bldg.append(outputs[1])

# Generating a time_array that matches the data for plotting
time_array = np.arange(start_time, (start_time + run_time), dt)

# Plots the building temperature
fig, axes = plt.subplots(1, 2, figsize=(12,5))
ax1 = axes[0]
ax1.plot(time_array, np.array(T_bldg).flatten(), '-b')
ax1.legend(['Zone Temp'])
xticks = ax1.get_xticks()
ax1.set_xticklabels(['{:02.0f}:{:02.0f}'.format(*divmod(val, 60)) for val in xticks])
ax1.set_title('Building Temperature')
ax1.set_ylabel('Temperature [C]')
ax1.set_xlabel('Time')

# Plots the building power
ax2 = axes[1]
ax2.plot(time_array, np.array(P_bldg).flatten(), '-r')
ax2.legend(['Bldg Power'])
xticks = ax2.get_xticks()
ax2.set_xticklabels(['{:02.0f}:{:02.0f}'.format(*divmod(val, 60)) for val in xticks])
ax2.set_title('Building Power')
ax2.set_ylabel('Power [kw]')
ax2.set_xlabel('Time')

# Displays the plots
plt.show()