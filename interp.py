from scipy.interpolate import interp1d as interp1d
import pandas as pd
import numpy as np

file_name = 'input/ROM_simulation_data.csv'

data = pd.read_csv(file_name)

time = np.arange(0, len(data)*5, 5)
Toa = data['T_outside'].values
Q_int = data['Q_internal'].values
Q_sol = data['Q_solar'].values
Q_hvac = data['Q_hvac'].values
T_room_rom = data['T_room_rom'].values
T_room_EP = data['T_room_EP'].values

time_interp = np.arange(0, (len(data)-1)*5, 1)

print(np.max(time))
print(np.max(time_interp))

f_Toa_interp = interp1d(time, Toa)
f_Q_int_interp = interp1d(time, Q_int)
f_Q_sol_interp = interp1d(time, Q_sol)
f_Q_hvac_interp = interp1d(time, Q_hvac)
f_T_room_rom_interp = interp1d(time, T_room_rom)
f_T_room_EP_interp = interp1d(time, T_room_EP)

Toa_interp = f_Toa_interp(time_interp)
Q_int_interp = f_Q_int_interp(time_interp)
Q_sol_interp = f_Q_sol_interp(time_interp)
Q_hvac_interp = f_Q_hvac_interp(time_interp)
T_room_rom_interp = f_T_room_rom_interp(time_interp)
T_room_EP_interp = f_T_room_EP_interp(time_interp)

print(data)
print(Toa_interp[0:11])

data_interp = pd.DataFrame()
data_interp['T_outside'] = Toa_interp
data_interp['Q_internal'] = Q_int_interp
data_interp['Q_solar'] = Q_sol_interp
data_interp['Q_hvac'] = Q_hvac_interp
data_interp['T_room_rom'] = T_room_rom_interp
data_interp['T_room_EP'] = T_room_EP_interp

data_interp.to_csv('ROM_simulation_data_interp.csv')