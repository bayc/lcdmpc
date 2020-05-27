# Copyright 2020 NREL

# Licensed under the Apache License, Version 2.0 (the "License"); you may not 
# use this file except in compliance with the License. You may obtain a copy of 
# the License at http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software 
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT 
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the 
# license for the specific language governing permissions and limitations under 
# the License.

import numpy as np
import autograd.numpy as np  # thinly-wrapped numpy
from autograd import grad    # the only autograd function you may ever need
from copy import deepcopy
from scipy.linalg import block_diag

from floris.floris import Floris
from floris.flow_field import FlowField

class wind_mdl:
    def __init__(self, input_file, yaw, Ct, ws, wd, horiz_len):
        self.inputs = [yaw, Ct, ws, wd]
        self.input_file = input_file
        self.power_scale = 1e1
        self.horiz_len = horiz_len

        self.yaw_min = 0.1
        self.yaw_max = 25.0
        self.Ct_min = 0.01
        self.Ct_max = 0.99

        self.reinit(self.inputs)

    def reinit(self, inputs):
        yaw = inputs[0]
        Ct = inputs[1]
        ws = inputs[2]
        wd = inputs[3]

        self.floris_setup(yaw, Ct, ws, wd)

        self.max_Ct = self.floris_get_max_Ct(yaw, ws, wd)

        print('max_Ct: ', self.max_Ct)
        # print('Yaw values: ', yaw)
        # print('Yaw times horiz: ', yaw*self.horiz_len)
        # print('Ct values: ', Ct)
        # print('Ct times horiz: ', Ct*self.horiz_len)
        self.var_keys = ['yaw_r', 'Ct_r']
        self.var_group_init = {
            'yaw_r' : {
                'lower' : [-2.5 if val > 2.5 \
                          else val-1.0 for val in yaw]*self.horiz_len,
                'upper' : [2.5 if val < 5. \
                          else 7.5-val for val in yaw]*self.horiz_len,
                'value' : yaw*self.horiz_len
            },
            # 'Ct_r' : {
            #     'lower' : [-val + 0.15 if val > 0.15 \
            #                else 0.0 for val in Ct]*self.horiz_len,
            #     'upper' : [0.99 if Ct[i] < self.max_Ct[i] - 0.99 \
            #                else self.max_Ct[i]-Ct[i] for i in range(len(Ct))]*self.horiz_len,
            #     'value' : Ct*self.horiz_len
            # }
            'Ct_r' : {
                'lower' : [-val + 0.2 if val > 0.2 \
                           else 0.0 for val in Ct]*self.horiz_len,
                'upper' : [0.99 if Ct[i] < self.max_Ct[i] - 0.99 \
                           else self.max_Ct[i]-Ct[i] for i in range(len(Ct))]*self.horiz_len,
                'value' : Ct*self.horiz_len
            }
        }
        print(self.var_group_init)

        # Model matrices
        # a0 = np.tile([1. + k_theta, 0., 0., 0.], (self.nturbs, 1))
        # a1 = np.tile([1., 0., 0., 0.], (self.nturbs, 1))
        # a2 = np.tile([0., 0., 1. + k_aI, 0.], (self.nturbs, 1))
        # a3 = np.tile([0., 0., 1., 0.], (self.nturbs, 1))))

        # a = np.array([[1. + k_theta, 0., 0., 0.],
        #              [1., 0., 0., 0.],
        #              [0., 0., 1. + k_aI, 0.],
        #              [0., 0., 1., 0.]])

        k_theta = -0.9  # P gain for yaw controller
        k_aI = -0.5       # P gain for axial induction controller

        a = np.array([[1. + k_theta, 0.],
                     [0., 1. + k_aI]])

        self.A = block_diag(*([a] * self.nturbs))

        # self.A = np.concatenate((
        #                 np.tile([1. + k_theta, 0., 0., 0.], (self.nturbs, 1)),
        #                 np.tile([1., 0., 0., 0.], (self.nturbs, 1)),
        #                 np.tile([0., 0., 1. + k_aI, 0.], (self.nturbs, 1)),
        #                 np.tile([0., 0., 1., 0.], (self.nturbs, 1))))
        # self.Bu = np.concatenate((np.tile([-k_theta, 0.], (self.nturbs, 1)),
        #                 np.tile([0., 0.], (self.nturbs, 1)),
        #                 np.tile([0., -k_aI], (self.nturbs, 1)),
        #                 np.tile([0., 0.], (self.nturbs, 1))))
        # bu = np.array([[-k_theta, 0.],
        #                 [0., 0.],
        #                 [0., -k_aI],
        #                 [0., 0.]])
        bu = np.array([[-k_theta, 0.],
                        [0., -k_aI]])
        self.Bu = block_diag(*([bu] * self.nturbs))
        # self.Bv = np.zeros((4*self.nturbs, 1))
        # self.Bd = np.zeros((4*self.nturbs, 2))                     
        self.Bv = np.zeros((2*self.nturbs, 1))
        self.Bd = np.zeros((2*self.nturbs, 2))
        # self.Bd = np.concatenate((np.tile([0., 0.], (self.nturbs, 1)),
        #                 np.tile([0., 0.], (self.nturbs, 1)),
        #                 np.tile([0., 0.], (self.nturbs, 1)),
        #                 np.tile([0., 0.], (self.nturbs, 1))))

        farm_grad = self.grad_farm([yaw, Ct, ws, wd])
        print('gradient inputs: ', [yaw, Ct, ws, wd])
        print('Cy gradient: ', farm_grad)
        # self.Cy = np.array([np.concatenate((np.array(farm_grad[0]), 
        #                     np.zeros(self.nturbs), 
        #                     np.array(farm_grad[1]), 
        #                     np.zeros(self.nturbs)))])
        # self.Cy = np.array([np.ndarray.flatten(np.array([np.array((x, 0, y, 0)) 
        # for x,y in zip(farm_grad[0], farm_grad[1])]))])

        self.Cy = np.array([np.ndarray.flatten(np.array([np.array((x, y)) 
        for x,y in zip(farm_grad[0], farm_grad[1])]))])

        self.Dyu = np.array([np.zeros(2*self.nturbs)])     # should be zeros
        self.Dyv = np.array([[0.0]])        # will eventually be setup for
                                            # building powers
        self.Dyd = np.array([[farm_grad[2], farm_grad[3]]])   # from autograd

        # Make same as y counterparts
        self.Cz = deepcopy(self.Cy)
        self.Dzu = deepcopy(self.Dyu)
        self.Dzv = deepcopy(self.Dyv)
        self.Dzd = deepcopy(self.Dyd)

        # self.update_var_group_init()

    def parse_opt_vars(self, varDict):
        self.yaw_r = varDict['yaw_r']
        self.Ct_r = varDict['Ct_r']

        vars = [self.yaw_r, self.Ct_r]
        return [val for tup in zip(*vars) for val in tup]

    def parse_sol_vars(self, sol):
        self.yaw_r = list(sol.getDVs().values())[0]
        self.Ct_r = list(sol.getDVs().values())[1]

        vars = [self.yaw_r, self.Ct_r]
        return [val for tup in zip(*vars) for val in tup]

    def update_inputs(self, x0):
        yaw = list(x0[0::2] + self.inputs[0])
        Ct = list(x0[1::2] + self.inputs[1])
        # ws = d[0]
        # wd = d[1]
        # self.inputs = [yaw, Ct, ws, wd]
        self.inputs = [yaw, Ct]

        print('new inputs for linearization: ', self.inputs)
        # return self.inputs

    def update_disturbances(self, d):
        ws = d[0]
        wd = d[1]
        self.d = [ws, wd]

    def update_var_group_init(self, var, lower, upper, value):
        self.var_group_init[var]['lower'] = lower
        self.var_group_init[var]['upper'] = upper
        self.var_group_init[var]['value'] = value

    def add_var_group(self, optProb):
        # optProb.addVarGroup('yaw_r', self.nturbs*self.horiz_len, type='c', 
        #                     lower=-0.4, upper=5.0, value=0.05)
        # optProb.addVarGroup('Ct_r', self.nturbs*self.horiz_len, type='c', 
        #                     lower=-0.03, upper=0.03, value=0.01)
        optProb.addVarGroup('yaw_r', self.nturbs*self.horiz_len, type='c', 
                            lower=self.var_group_init['yaw_r']['lower'], 
                            upper=self.var_group_init['yaw_r']['upper'], 
                            value=self.var_group_init['yaw_r']['value'],
                            scale=1)
        optProb.addVarGroup('Ct_r', self.nturbs*self.horiz_len, type='c', 
                            lower=self.var_group_init['Ct_r']['lower'], 
                            upper=self.var_group_init['Ct_r']['upper'], 
                            value=self.var_group_init['Ct_r']['value'],
                            scale=1)
        return optProb

    def add_con_group(self, optProb):
        # optProb.addConGroup('hvac_con', self.horiz_len, lower=0, upper=0)
        # optProb.addConGroup('T_building_con', self.horiz_len, lower=-2, upper=2)

        return optProb

    def compute_cons(self, funcs, uOpt, Y):
        # Add constraints for the SNOPT optimization
        # Qhvac = uOpt[0::3]
        # ms_dot = uOpt[1::3]
        # T_sa = uOpt[2::3]

        # funcs['hvac_con'] = np.array(Qhvac) - np.array(ms_dot)*(np.array(T_sa) \
        #                 - (Y + 23.56))
        # funcs['T_building_con'] = Y

        return funcs

    def floris_get_max_Ct(self, yaw, ws, wd):
        self.floris_change_ws_wd(ws, wd)

        for i, (coord, turbine) \
                in enumerate(self.floris.farm.turbine_map.items()):

                turbine.Ct_control = None

        self.floris.farm.flow_field.calculate_wake()
        return [turb.Ct for turb in self.floris.farm.turbine_map.turbs()]


    def floris_setup(self, yaw, Ct, ws, wd):
        self.floris = Floris(self.input_file)
        self.nturbs = len(self.floris.farm.turbine_map.turbs())

        self.floris_change_ws_wd(ws, wd)

        self.floris.farm.flow_field.calculate_wake()

        power0 = []
        for i, (coord,turbine) in enumerate(self.floris.farm.turbine_map.items()):
            power0.append(turbine._calculate_power())
        # print('total power: ', np.sum(np.array(power0)))

        self.power_initial = self.floris_get_power(yaw, Ct, ws, wd)
        # print('POWER: ', self.power_initial)
        # print('Initial wind model power: {:3.2f} MW'.format(self.power_initial))

        self.grad_farm_yaw = grad(self.floris_get_farm_gradient_yaw)
        self.grad_farm_Ct = grad(self.floris_get_farm_gradient_Ct)
        self.grad_farm_ws = grad(self.floris_get_farm_gradient_ws)
        self.grad_farm_wd = grad(self.floris_get_farm_gradient_wd)
        self.grad_farm = grad(self.floris_get_farm_gradient)
        # print('yaw grad: ', self.grad_farm_yaw(yaw, Ct, ws, wd))
        # print('Ct grad: ', self.grad_farm_Ct(Ct, yaw, ws, wd))
        # print('ws grad: ', self.grad_farm_ws(ws, yaw, Ct, wd))
        # print('wd grad: ', self.grad_farm_wd(wd, yaw, Ct, ws))
        # print('farm grad: ', self.grad_farm([yaw, Ct, ws, wd]))

    def floris_get_farm_gradient_yaw(self, yaw, Ct, ws, wd):
        return self.floris_get_power(yaw, Ct, ws, wd)/self.power_scale

    def floris_get_farm_gradient_Ct(self, Ct, yaw, ws, wd):
        return self.floris_get_power(yaw, Ct, ws, wd)/self.power_scale

    def floris_get_farm_gradient_ws(self, ws, yaw, Ct, wd):
        return self.floris_get_power(yaw, Ct, ws, wd)/self.power_scale

    def floris_get_farm_gradient_wd(self, wd, yaw, Ct, ws):
        return self.floris_get_power(yaw, Ct, ws, wd)/self.power_scale

    def floris_get_farm_gradient(self, inputs):
        yaw = inputs[0]
        Ct = inputs[1]
        ws = inputs[2]
        wd = inputs[3]

        return self.floris_get_power(yaw, Ct, ws, wd)/self.power_scale

    def floris_get_power(self, yaw, Ct, ws, wd):
        self.floris_change_yaw(yaw)
        self.floris_change_Ct(Ct)
        self.floris_change_ws_wd(ws, wd)

        self.floris.farm.flow_field.calculate_wake()

        power0 = []
        for i, (coord,turbine) in enumerate(
            self.floris.farm.turbine_map.items()
            ):
            power0.append(turbine._calculate_power())
        # print('total power: ', np.sum(np.array(power0)))
        return np.sum(np.array(power0))/self.power_scale

    def floris_change_yaw(self, yaw):
        if yaw is not None:
            for i, (coord, turbine) in \
                enumerate(self.floris.farm.turbine_map.items()):

                turbine.yaw_angle = np.radians(yaw[i])

    def floris_change_Ct(self, Ct):
        if Ct is not None:
            for i, (coord, turbine) \
                in enumerate(self.floris.farm.turbine_map.items()):

                turbine.Ct_control = Ct[i]

    def floris_change_ws_wd(self, ws, wd):
        # Frame of reference is west (270 deg)
        self.floris.farm.flow_field.wind_direction = np.radians(wd - 270) 
        self.floris.farm.flow_field.wind_speed = ws
        self.floris.farm.flow_field.initial_flowfield = \
            self.floris.farm.flow_field._initial_flowfield()
        self.floris.farm.flow_field.u_field = \
            self.floris.farm.flow_field._initial_flowfield()

    # def add_cons(self, funcs, uOpt, Y):
    #     # Add constraints for the SNOPT optimization
    #     Qhvac = uOpt[0::3]
    #     ms_dot = uOpt[1::3]
    #     T_sa = uOpt[2::3]

    #     funcs['con1'] = np.array(Qhvac) \
    #                     - np.array(ms_dot)*(np.array(T_sa) - (Y + 23.56))
    #     funcs['con6'] = Y

    #     return funcs

