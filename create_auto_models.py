import pandas as pd
from pathlib import Path
import shutil
import fileinput
import os
import subprocess


cases_directory = 'models/model_identification/'

control_model_template_name = 'models/control/bldg_template_control.py'
sim_model_template_name = 'models/simulation/bldg_template_sim.py'

control_model_base_name = 'bldg_ctrl_pena_'
sim_model_base_name = 'bldg_sim_pena_'

for model in os.listdir(cases_directory):
    if model.endswith('.xlsx'):
        print(cases_directory + model)
        # print(model.strip('building_').strip('.xlsx'))
        
        df_model = pd.read_excel(cases_directory + model, engine='openpyxl')
        print(df_model)

        bldg_num = int(df_model['Building Number'][0])
        # bldg_num = model.strip('building_').strip('.xlsx')

        a1 = df_model['Fan Power'][2]
        a2 = df_model['Fan Power'][1]
        a3 = df_model['Fan Power'][0]

        hvac_cop_inv = df_model['Chiller Power'][0]

        T_sa = df_model['T discharge limits']

        A = df_model['A'][0]
        Bu = df_model['Bu'][0]
        Bd = df_model['Bd']
        K = df_model['K'][0]
        Bd_mean_inputs = df_model['Means'][0]
        Cy_mean_outputs = df_model['Means'][1]

        C_r_inv = df_model['3r2c_model'][0]
        C_e_inv = df_model['3r2c_model'][1]
        R_re_inv = df_model['3r2c_model'][2]
        R_ra_inv = df_model['3r2c_model'][3]
        R_ea_inv = df_model['3r2c_model'][4]

        new_control_model_name = 'models/control/' + control_model_base_name + str(bldg_num) + '.py' 
        new_sim_model_name = 'models/simulation/' + sim_model_base_name + str(bldg_num) + '.py' 

        shutil.copy(control_model_template_name, new_control_model_name)
        shutil.copy(sim_model_template_name, new_sim_model_name)

        control_model_dict = {
            '        self.a1 = None': '        self.a1 = ' + str(a1),
            '        self.a2 = None': '        self.a2 = ' + str(a2),
            '        self.a3 = None': '        self.a3 = ' + str(a3),
            '        self.hvac_cop_inv = None': '        self.hvac_cop_inv = ' + str(hvac_cop_inv),
            '        self.A = None': '        self.A = ' + 'np.array([[' + str(A) + ']])',
            '        self.Bu = None': '        self.Bu = ' + 'np.array([[' + str(Bu) + ', 0.0, 0.0]])',
            '        self.Bd = None': '        self.Bd = ' + 'np.array([[' + str(Bd[0]) + ', ' + str(Bd[1]) + ', ' + str(Bd[2]) + ', 0.0]])',
            '        self.K = None': '        self.K = ' + str(K),
            '        self.Bd_mean_inputs = np.array([None, [0.0], [0.0], [0.0]])': '        self.Bd_mean_inputs = ' + 'np.array([[' + str(Bd_mean_inputs) + '], [0.0], [0.0], [0.0]])',
            '            None,': '            [' + str(Cy_mean_outputs) + '],',
            '                            lower=None, upper=None, value=12.8)': '                            lower=' + str(T_sa[0]) + ', upper=' + str(T_sa[1]) + ', value=12.8)',

        }

        for line in fileinput.input(new_control_model_name, inplace=True):
            line = line.rstrip('\r\n')
            print(control_model_dict.get(line, line))

        simulation_model_dict = {
            '        self.a1 = None': '        self.a1 = ' + str(a1),
            '        self.a2 = None': '        self.a2 = ' + str(a2),
            '        self.a3 = None': '        self.a3 = ' + str(a3),
            '        self.hvac_cop_inv = None': '        self.hvac_cop_inv = ' + str(hvac_cop_inv),
            '        self.C_r_inv = np.array([[None]]) # room air capacitance inverse': '        self.C_r_inv = np.array([[1/' + str(C_r_inv) + ']]) # room air capacitance inverse',
            '        self.C_e_inv = np.array([[None]]) # external wall equivalent cpaacitance inverse': '        self.C_e_inv = np.array([[1/' + str(C_e_inv) + ']]) # external wall equivalent cpaacitance inverse',
            '        self.R_re_inv = np.array([[None]]) # room air to wall resistance inverse': '        self.R_re_inv = np.array([[1/' + str(R_re_inv) + ']]) # room air to wall resistance inverse',
            '        self.R_ra_inv = np.array([[None]]) # room air to outside air resistance inverse': '        self.R_ra_inv = np.array([[1/' + str(R_ra_inv) + ']]) # room air to outside air resistance inverse',
            '        self.R_ea_inv = np.array([[None]]) # external wal to outside air resistance inverse': '        self.R_ea_inv = np.array([[1/' + str(R_ea_inv) + ']]) # external wal to outside air resistance inverse',
        }

        for line in fileinput.input(new_sim_model_name, inplace=True):
            line = line.rstrip('\r\n')
            print(simulation_model_dict.get(line, line))


