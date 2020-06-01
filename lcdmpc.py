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
import pandas as pd
import copy
from numpy import dot as dot
from numpy import transpose as tp
from numpy.linalg import matrix_power as matpower
from pyoptsparse import Optimization, SNOPT

class LCDMPC():
    def __init__(self, start_time, time_step):
        self.subsystems = []
        self.current_time = start_time
        self.time_step = time_step

    def build_subsystem(self, control_model, truth_model, inputs, outputs,
        refs, horiz_len, Beta, disturbance_file, nodeID=None, nodeName=None):
        # create subsystem object
        subsys = subsystem(self, control_model, truth_model, inputs,
            outputs, refs, horiz_len, Beta, disturbance_file, self.current_time, nodeID=nodeID, nodeName=nodeName)
        # append it to subsystem list
        self.subsystems.append(subsys)

    def simulate_truth_model(self):
        # apply control action and simulate truth models
        
        for subsys in self.subsystems:
            u_applied = subsys.uConv[:subsys.control_model.numDVs]
            print('u_applied: ', u_applied)
            disturb = subsys.d
            outputs = subsys.simulate_truth(self.current_time, u_applied, disturb)
            print('outputs: ', outputs)

        # TODO: change to date-time; update in other places
        self.current_time = self.current_time + 1

        return outputs

    def update_control_filter(self):
        for subsys in self.subsystems:
            subsys.update_control_filter()

    def update_forecast_inputs(self):
        # update the forecasts for each of the subsystem models

        for subsys in self.subsystems:
            subsys.update_forecast(self.current_time)

    def optimize(self):
        pass

    def reinitialize(self):
        pass

    def build_interconnections(self, interconnections):
        """
        Accept inteconnections from subsystems and setup 
        interconnection. matrix
        
        Args:
            interconnections (list): A list of tuples containing the 
                upstream and downstream nodeIds.
        """
        if interconnections==None:
            for subsys in self.subsystems:
                subsys.downstream.append('None')
                subsys.upstream.append('None')
        else:
            for pair in interconnections:
                up = pair[0]
                down = pair[1]

                self.subsystems[up].downstream.append(down)
                self.subsystems[down].upstream.append(up)

    def update_downstream_outputs(self):
        """
        Update the z vectors of the subsystems.
        """
        for subsys in self.subsystems:
            subsys.update_Z()
            print('self.Z: ', subsys.Z)

    def communicate(self):
        """
        Exchange information between subsystems.
        """
        for subsys in self.subsystems:
            subsys.update_V(self)
            subsys.update_Psi(self)

    def optimize_all(self):
        """
        Loop through all subsystems and perform optimization.
        """
        for subsys in self.subsystems:
            subsys.uOpt = subsys.optimize()

    def convex_sum_cont(self):
        """
        Perform the convex combination of the new control action and 
        the previous control action.
        """
        for subsys in self.subsystems:
            subsys.uConv = subsys.Beta*np.array(subsys.uConv) + \
                           (1 - subsys.Beta)*np.array(subsys.uOpt)
            subsys.uConv[0::3] = np.array(subsys.uConv[1::3])*(np.array(subsys.uConv[2::3]) \
                        - (subsys.control_model.truth_model_T_z))
            # subsys.uConv[1::3] = 1
            # subsys.uConv = subsys.uOpt

    def calculate_sensitivities(self):
        """
        Calculate the sensitivities of the subsystem to the upstream 
        systems.
        """
        for subsys in self.subsystems:
            subsys.calc_sens()

    def calc_obj(self):
        for subsys in self.subsystems:
            subsys.calc_obj()

    def update_states(self):
        """
        Update the states of the subsystems.
        """
        for subsys in self.subsystems:
            subsys.update_x()

    def update_subsystem_outputs(self):
        """
        Update the subsystem outputs, 'y'.
        """
        for subsys in self.subsystems:
            subsys.update_y()

    def update_inputs_for_linearization(self):
        """
        Updates the inputs for the subsystems to correctly relinearize, if 
        needed.
        """
        for subsys in self.subsystems:
            subsys.update_inputs()
            # subsys.update_disturbances()

    def relinearize_subsystem_models(self):
        """
        Uses the latest input data to relinearize the subsystem models.
        """
        for subsys in self.subsystems:
            subsys.relinearize(subsys.control_model, subsys.inputs, 
                               subsys.d, subsys.outputs)


class subsystem():
    def __init__(self, obj, control_model, truth_model, inputs, outputs, refs, horiz_len, 
                 Beta, disturbance_file, current_time, upstream=None, downstream=None, 
                 nodeID=None, nodeName=None, optOptions=None):
        # self.Q = Q
        # self.S = S
        # self.count = 0
        self.current_time = current_time
        
        self.truth_model = truth_model
        self.control_model = control_model
        self.refs = refs 
        self.refs_const = copy.deepcopy(self.refs)
        self.refs = self.refs - self.control_model.truth_model_Pwr \
            + self.control_model.Cy_lin \
            + self.control_model.Dyu_lin \
            + self.control_model.Dyd_lin
        # print('initial refs: ', self.refs)
        self.horiz_len = horiz_len
        self.Beta = Beta

        self.disturbance_file = disturbance_file
        self.disturbance_data = pd.read_csv(self.disturbance_file)
        self.update_forecast(current_time)

        self.y = self.control_model.Cy_mean_outputs
        self.refs = self.refs*self.horiz_len
        self.refs_const = self.refs_const*self.horiz_len
        print('refs: ', self.refs)
        self.relinearize(control_model, inputs, self.d, outputs)

        if upstream is not None:
            self.upstream = upstream
        else:
            self.upstream = []
        if downstream is not None:
            self.downstream = downstream
        else:
            self.downstream = []
        if nodeID is not None:
            self.nodeID = nodeID
        else:
            self.nodeID = str(len(obj.subsystems))
        if nodeName is not None:
            self.nodeName = nodeName
        else:
            self.nodeName = 'Node ' + str(len(obj.subsystems))
        if optOptions is not None:
            self.optOptions = optOptions
        else:
            self.optOptions = {'Major feasibility tolerance' : 1e-1}

        self.x0 = np.zeros(self.nxA)
        # self.x0 = np.array([[0.0] for _ in range(self.nxA)])
        self.x1 = []
        self.u0 = np.zeros(self.nyBu*self.horiz_len)
        self.uConv = self.u0
        self.u0_min = 0.0
        self.u0_max = 100.0
        # self.uOpt = []
        self.uOpt = [-100, 0.8, 12.8]*self.horiz_len
        self.V = np.zeros(self.nyNy)
        self.D = np.zeros(self.nyPy)
        # self.V = np.array([[0.0] for _ in range(self.nyNy)])
        
        self.Z = np.zeros(self.nxNz)
        self.Gamma = np.zeros(self.nxNz)
        self.Psi = np.zeros(self.nxMz)
        # print('initial horizon refs: ', self.refs)
        self.sol = []

    def update_control_filter(self):
        print('self.y before filter: ', self.y)
        self.x0 = self.control_model.filter_update(self.x0, self.truth_model.outputs)
        print('self.x0 after filter update: ', self.x0)

    def simulate_truth(self, current_time, inputs, disturb):
        self.current_time = current_time
        outputs = self.truth_model.simulate(current_time, inputs, disturb)
        print('outputs: ', outputs)

        return outputs

    def update_forecast(self, current_time):
        # disturbance horizon
        self.d = self.truth_model.get_forecast(current_time, self.disturbance_data)
        print('self.d: ', self.d)
        self.D = self.control_model.get_forecast(current_time, self.disturbance_data)
        print('self.D: ', self.D)

    def relinearize(self, control_model, inputs, disturb, outputs):

        # self.refs[1::2] = [self.refs_const[1] - control_model.truth_model_Pwr]*self.horiz_len
        # print('power refs: ', self.refs[1::2])
        # import matplotlib.pyplot as plt
        # plt.figure()
        # plt.plot(self.refs)
        # plt.show()
        # lkj
        # print('self.refs before: ', self.refs)
        # print('dy: ', (self.y[0] - self.control_model.Cy_mean_outputs))
        # self.refs = [val - (self.y[0] - self.control_model.Cy_mean_outputs) for val in self.refs_const]
        # print('self.refs after: ', self.refs)

        self.inputs = inputs
        self.outputs = outputs

        self.control_model = control_model

        control_model.reinit(inputs, disturb)

        if self.current_time == 165:
            self.refs_const[1::2] = [self.refs_const[1] + 20]*self.horiz_len
        if self.current_time == 180:
            self.refs_const[1::2] = [self.refs_const[1] - 10]*self.horiz_len

        self.refs = np.array(self.refs_const) \
            - np.array([0, self.control_model.truth_model_Pwr]*self.horiz_len) \
            + np.array([self.control_model.Cy_lin]*self.horiz_len).flatten() \
            + np.array([self.control_model.Dyu_lin]*self.horiz_len).flatten() \
            + np.array([self.control_model.Dyd_lin]*self.horiz_len).flatten()
        print('**********Refs: ', self.refs)

        self.A = control_model.A
        self.Bu = control_model.Bu
        self.Bv = control_model.Bv
        self.Bd = control_model.Bd
        self.Cy = control_model.Cy
        self.Cz = control_model.Cz
        self.Dyu = control_model.Dyu
        self.Dyv = control_model.Dyv
        self.Dyd = control_model.Dyd
        self.Dzu = control_model.Dzu
        self.Dzv = control_model.Dzv
        self.Dzd = control_model.Dzd

        self.mat_sizes()
        self.sys_matrices()

        print('size My: ', self.nxMy, self.nyMy)
        print('size Ny: ', self.nxNy, self.nyNy)
        print('size Py: ', self.nxPy, self.nyPy)
        # lkj

        self.E = dot(dot(tp(self.Ny), self.Q), self.Ny)

        self.d = disturb

    def calculate_horizon(self):
        # for building model, assume room_temp measurement
        # to be last measurement over horizon
        self.update_Y()

    def set_opt_bounds(self, low, high):
        pass

    def optimize(self):
        self.update()

        optProb = Optimization('LCDMPC', self.obj_func)
        # optProb.addVarGroup('U', len(self.u0), type='c', lower=self.u0_min, 
        #                     upper=self.u0_max, value=self.u0)

        optProb = self.control_model.add_var_group(optProb)

        optProb = self.control_model.add_con_group(optProb)

        optProb.addObj('obj')

        # optProb = self.add_constraints(optProb)

        opt = SNOPT(optOptions=self.optOptions)
        sol = opt(optProb, sens=self.sens)
        # sol = opt(optProb, sens='FDR')
        # sol = opt(optProb)

        self.sol = sol

        # print(sol)

        self.uOpt = self.control_model.parse_sol_vars(sol)
        return self.uOpt

    def obj_func(self, varDict):
        # Parse the optimization variables defined in the control model file
        self.uOpt = self.control_model.parse_opt_vars(varDict)
        
        # Setup the objective function for LC-DMPC
        funcs = {}
        funcs['obj'] = (dot(dot(tp(self.uOpt), self.H), self.uOpt) \
                     + 2*dot(tp(self.uOpt), self.F) \
                     + dot(dot(tp(self.V), self.E), self.V) \
                     + 2*dot(tp(self.V), self.T))

        # Compute constraints, if any are defined for control model
        self.update_Y()
        # print('self.Y: ', self.Y)
        funcs = self.control_model.compute_cons(funcs, self.uOpt, self.Y)

        fail = False
        return funcs, fail

    def calc_obj(self):

        obj_func = (dot(dot(tp(self.uOpt), self.H), self.uOpt) \
                     + 2*dot(tp(self.uOpt), self.F) \
                     + dot(dot(tp(self.V), self.E), self.V) \
                     + 2*dot(tp(self.V), self.T))
        # print('opt_control: ', self.uConv[:self.control_model.numDVs])
        # print('objective function: ', obj_func)
        print('self.Y: ', self.Y)
        print('self.Dy: ', self.control_model.Cy)
        print('self.My:', self.Fy)
        
        # print('********************UPDATE_Y: ', self.uConv)
        # print('UPDATEY SELF.D: ', self.D - np.tile(
        #             self.control_model.Bd_mean_inputs, self.horiz_len
        #         ))
        print('UPDATEY uconv: ', self.V)
        # print('self.Psi: ', self.Psi)
        # print('self.V: ', self.V)

    def sens(self, varDict, funcs):
        # Parse the optimization variables defined in the control model file
        self.uOpt = self.control_model.parse_opt_vars(varDict)

        funcsSens = {}
        # funcsSens = {
        #     ('obj', 'yaw_r') : (2*dot(self.H, self.uOpt) + 2*self.F)[0::2],
        #     ('obj', 'Ct_r') : (2*dot(self.H, self.uOpt) + 2*self.F)[1::2]
        # }
        # a = np.array(self.uOpt[1::3]) + self.truth_model.T_z
        # b = self.truth_model.T_z
        # b = (a.diagonal() + self.truth_model.T_z)
        # print('a: ', self.uOpt[2::3] - self.truth_model.T_z.flatten())
        # print('b: ', -1*np.diag(self.uOpt[2::3] - self.truth_model.T_z.flatten()))
        # print(np.diag(b))
        # print('shape: ', np.diag(np.ones(self.horiz_len)))
        # print(type(self.truth_model.T_z))
        # print('stuff: ', np.fill_diagonal(a, a.diagonal() + self.truth_model.T_z))
        # np.fill_diagonal(a, a.diagonal() / c)
        funcsSens = {
            ('obj', 'Qhvac') : (2*dot(self.H, self.uOpt) + 2*self.F)[0::3],
            ('obj', 'ms_dot') : (2*dot(self.H, self.uOpt) + 2*self.F)[1::3],
            ('obj', 'T_sa') : (2*dot(self.H, self.uOpt) + 2*self.F)[2::3],
            ('hvac_con', 'Qhvac') : np.diag(np.ones(self.horiz_len)),
            ('hvac_con', 'ms_dot') : -1*np.diag(self.uOpt[2::3] - self.truth_model.T_z.flatten()),
            ('hvac_con', 'T_sa') : np.diag(-1*np.array(self.uOpt[1::3])),
            ('T_building_con', 'Qhvac') : self.My[:self.horiz_len, 0::3],
            ('T_building_con', 'ms_dot') : self.My[:self.horiz_len, 1::3],
            ('T_building_con', 'T_sa') : self.My[:self.horiz_len, 2::3]}
        #     #  ('con1', 'Qhvac') : np.ones(5)*1.0,
        #     #  ('con1', 'ms_dot') : -1*np.ones(5)*self.T_sa[0],
        #     #  ('con1', 'T_sa') : -1*np.ones(5)*self.ms_dot[0]}
        #     ('con1', 'Qhvac') : np.eye(5)*1.0,
        #     ('con1', 'ms_dot') : -1*np.eye(5)*self.T_sa,
        #     ('con1', 'T_sa') : -1*np.eye(5)*self.ms_dot,
        #     ('con6', 'Qhvac') : np.eye(5)*np.diag(self.model.A),
        #     ('con6', 'ms_dot') : np.eye(5)*0.0,
        #     ('con6', 'T_sa') : np.eye(5)*1.0}
        #     ('con6', 'ms_dot') : np.diag(
        #         (3*self.model.a0*self.ms_dot**2 \
        #         + 2*self.model.a1*self.ms_dot \
        #         + self.model.a0 + 1.005/self.model.hvac_cop \
        #         * (0.3*self.model.T_oa + (1 - 0.3)*self.Y - self.T_sa))._value
        #     ),
        #     ('con6', 'T_sa'): np.diag(
        #         -1*1.005/self.model.hvac_cop*self.ms_dot
        #     )}
        #     ('con2', 'Qhvac') : 1.0,
        #     ('con2', 'ms_dot') : -1*self.T_sa[1],
        #     ('con2', 'T_sa') : -1*self.ms_dot[1],
        #     ('con3', 'Qhvac') : 1.0,
        #     ('con3', 'ms_dot') : -1*self.T_sa[2],
        #     ('con3', 'T_sa') : -1*self.ms_dot[2],
        #     ('con4', 'Qhvac') : 1.0,
        #     ('con4', 'ms_dot') : -1*self.T_sa[3],
        #     ('con4', 'T_sa') : -1*self.ms_dot[3],
        #     ('con5', 'Qhvac') : 1.0,
        #     ('con5', 'ms_dot') : -1*self.T_sa[4],
        #     ('con5', 'T_sa') : -1*self.ms_dot[4]}

        fail = False
        return funcsSens, fail

    # def add_constraints(self, optProb):
    #     for con in self.cons:
    #         optProb.addConGroup(con['name'], con['ncon'], con['lower'], 
    #                     con['upper'], con['wrt'], con['linear'], con['jac'])

    #     return optProb

    def update(self):
        print('///////////////// ERROR: ', (dot(self.Fy, self.x0) \
               + dot(self.Ny, self.V) + dot(self.Py, self.D - np.tile(
                    self.control_model.Bd_mean_inputs, self.horiz_len
                )) + dot(self.My, self.uOpt) - self.refs))
        print('rrrrrrrrrrrrrrrrrr refs: ', self.refs)
        # print('wwwwwwwwwwwwwwwww Y in obj: ', dot(self.Fy, self.x0) \
        #         + dot(self.Ny, self.V) + dot(self.Py, self.D - np.tile(
        #             self.control_model.Bd_mean_inputs, self.horiz_len
        #         )) + dot(self.My, self.uOpt) \
        #         + np.tile(self.control_model.Cy_mean_outputs, self.horiz_len) \
        #         - np.tile(self.control_model.Cy_lin, self.horiz_len) \
        #         - np.tile(self.control_model.Dyu_lin, self.horiz_len) \
        #         - np.tile(self.control_model.Dyd_lin, self.horiz_len))
        self.H = dot(dot(tp(self.My), self.Q), self.My) + self.S
        self.E_1 = self.E
        self.E = dot(dot(tp(self.Ny), self.Q), self.Ny)
        self.F = dot(dot(tp(self.My), self.Q), (dot(self.Fy, self.x0) \
               + dot(self.Ny, self.V) + dot(self.Py, self.D - np.tile(
                    self.control_model.Bd_mean_inputs, self.horiz_len
                )) - self.refs)) \
               + 0.5*dot(tp(self.Mz), self.Psi)
        self.T = dot(dot(tp(self.Ny), self.Q), (dot(self.Fy, self.x0) \
               + dot(self.Py, self.D - np.tile(
                    self.control_model.Bd_mean_inputs, self.horiz_len
                )) - self.refs)) + 0.5*dot(tp(self.Nz), self.Psi)

    def calc_sens(self):
        self.gamma = 2*(dot(self.E_1, self.V) + self.T \
                   + dot(dot(dot(tp(self.Ny), self.Q), self.My), self.uConv))

    def update_inputs(self):
        self.inputs = self.control_model.update_inputs(
            self.x0,
            self.uConv[:self.control_model.numDVs],
            self.truth_model.outputs
        )
        print('updated inputs: ', self.inputs)

    def update_disturbances(self):
        self.disturb = self.control_model.update_disturbances(self.d)

    def update_subsys(self):
        self.update_x()
        self.update_y()
        self.update_Z()

    def update_V(self, obj):
        # TODO: figure out how to make this work when z is more than one value
        for upstream in self.upstream:
            if upstream=='None':
                self.V = np.zeros(self.nyNy)
            else:
                self.V = obj.subsystems[upstream].Z
                # self.V = np.array([[val] for \
                #     val in obj.subsystems[upstream].Z])

    def update_Psi(self, obj):
        # TODO: figure out how to make this work when z is more than one value
        for upstream in self.upstream:
            if upstream=='None':
                self.Psi = np.zeros(self.nxMz)
            else:
                self.Psi = obj.subsystems[upstream].Gamma

    def update_x(self):
        # TODO: add in self.d to class
        self.x1 = dot(self.A, self.x0) \
            + dot(self.Bu, tp(np.array(self.uConv[0:self.nyBu]))) \
            + dot(self.Bv, self.V[0:len(self.upstream)]) + dot(self.Bd, self.d - self.control_model.Bd_mean_inputs)
        self.x0 = self.x1
        print('self.x0 after update_x: ', self.x0)

    def update_y(self):
        print('x0 statessssssssssssssssss: ', self.x0)
        print('********************UPDATE_y: ', tp(self.uConv[0:self.nyDyu]))
        # print('UPDATE y SELF.d: ', self.d - self.control_model.Bd_mean_inputs)
        self.y = dot(self.Cy, self.x0) \
            + dot(self.Dyu, tp(self.uConv[0:self.nyDyu])) \
            + dot(self.Dyv, self.V[0:len(self.upstream)]) \
            + dot(self.Dyd, self.d - self.control_model.Bd_mean_inputs) \
            + self.control_model.Cy_mean_outputs
        # self.y[1] = self.y[1] - dot(self.Dyu[1], tp(np.array([0., self.control_model.ms_dot_lin, self.control_model.T_sa_lin])))
        # self.y[1] = self.y[1] - dot(self.Dyd[1], np.array([self.control_model.T_oa_lin, 0., 0.]))
        # self.y[1] = self.y[1] - dot(self.Cy[1], self.control_model.T_z_lin)

        self.y = self.y - self.control_model.Cy_lin - self.control_model.Dyu_lin - self.control_model.Dyd_lin
        print('sadfasdfasd: ', self.y)

    def update_Y(self):
        # print('UPDATEY SELF.D: ', self.D - np.tile(
        #             self.control_model.Bd_mean_inputs, self.horiz_len
        #         ))
        self.Y = dot(self.Fy, self.x0) + dot(self.My, self.uOpt) \
            + dot(self.Ny, self.V) \
            + dot(
                self.Py, self.D - np.tile(
                    self.control_model.Bd_mean_inputs, self.horiz_len
                )
            ) \
            + np.tile(self.control_model.Cy_mean_outputs, self.horiz_len)
        self.Y = self.Y \
               - np.tile(self.control_model.Cy_lin, self.horiz_len) \
               - np.tile(self.control_model.Dyu_lin, self.horiz_len) \
               - np.tile(self.control_model.Dyd_lin, self.horiz_len)
        # print('shape Cy_lin: ', self.control_model.Cy_lin)
        # print('shape Dyu_lin: ', self.control_model.Dyu_lin)
        # print('shape Dyd_lin: ', self.control_model.Dyd_lin)

        #TODO: print system matrices and compare between Y and Z

    def update_Z(self):
        # print('UPDATEZ SELF.D: ', self.D - np.tile(
        #             self.control_model.Bd_mean_inputs, self.horiz_len
        #         ))
        print('UPDATEZ uconv: ', self.V)
        self.Z = dot(self.Fz, self.x0) + dot(self.Mz, self.uConv) \
            + dot(self.Nz, self.V) + dot(
                self.Pz, self.D - np.tile(
                    self.control_model.Bd_mean_inputs, self.horiz_len
                )
            ) \
            + np.tile(self.control_model.Cz_mean_outputs, self.horiz_len)
        # print('shape Cz_lin: ', self.control_model.Cz_lin)
        # print('shape Dzu_lin: ', self.control_model.Dzu_lin)
        # print('shape Dzd_lin: ', self.control_model.Dzd_lin)
        # print('shape of Z: ', np.shape(self.Z))
        self.Z = self.Z \
                - np.tile(self.control_model.Cz_lin, self.horiz_len) \
                - np.tile(self.control_model.Dzu_lin, self.horiz_len) \
                - np.tile(self.control_model.Dzd_lin, self.horiz_len)

    def sys_matrices(self):       
        # self.Fy = np.array([dot(self.Cy, matpower(self.A, i)) \
        #           for i in range(1, self.horiz_len + 1)])
        self.Fy = np.array([dot(self.Cy, matpower(self.A, i)) \
                  for i in range(0, self.horiz_len)])
        self.Fy = np.reshape(
            self.Fy, (self.nxCy*self.horiz_len, self.nxA), order='C'
        )
        # if self.Fy[0].ndim > 1:
        #     self.Fy = np.concatenate(self.Fy, axis=0)
        # else:
        #     self.Fy = np.concatenate(self.Fy)

        self.Fz = np.array([dot(self.Cz, matpower(self.A, i)) \
                  for i in range(0, self.horiz_len)])
        self.Fz = np.reshape(self.Fz, (self.nxCz*self.horiz_len, self.nxA))
        # if self.Fz[0].ndim > 1:
        #     self.Fz = np.concatenate(self.Fz, axis=1)
        # else:
        #     self.Fz = np.concatenate(self.Fz)

        # Mytmp = dot(self.Cy, self.Bu)
        # MytmpShape = np.shape(Mytmp)
        # for i in range(self.horiz_len - 1):
        #     Mytmp = np.hstack((Mytmp, np.zeros(MytmpShape)))
        # for i in range(1, self.horiz_len):
        #     if Mytmp.ndim == 1:
        #         Mytmp = np.vstack((Mytmp, np.hstack((dot(self.Cy, \
        #             dot(
        #                 matpower(self.A, i), self.Bu )), Mytmp[:-self.nyBu])))
        #             )
        #     else:
        #         if dot(self.Cy, dot(matpower(self.A, i), self.Bu )).ndim == 1:
        #             Mytmp = np.vstack((Mytmp, np.hstack((dot(self.Cy, \
        #                 dot(matpower(self.A, i), self.Bu )), 
        #                       Mytmp[-self.nxCy:,:-self.nyBu][0]))))
        #         else:
        #             Mytmp = np.vstack((Mytmp, np.hstack((dot(self.Cy, \
        #                 dot(matpower(self.A, i), self.Bu )), 
        #                       Mytmp[-self.nxCy:,:-self.nyBu]))))
        # self.My = Mytmp

        Mytmp0 = self.Dyu
        Mytmp0Shape = np.shape(Mytmp0)
        Mytmp = np.hstack((dot(self.Cy, self.Bu), Mytmp0))
        for i in range(self.horiz_len - 1):
            Mytmp0 = np.hstack((Mytmp0, np.zeros(Mytmp0Shape)))
        for i in range(self.horiz_len - 2):
            Mytmp = np.hstack((Mytmp, np.zeros(Mytmp0Shape)))
        Mytmp = np.vstack((Mytmp0, Mytmp))
        # Mytmp = dot(self.Cy, self.Bu)
        # # print('Mytmp: ', Mytmp)
        # MytmpShape = np.shape(Mytmp)
        # Mytmp = np.hstack((Mytmp, self.Dyu))
        # # print('(((((((((((((((( combo of CyBU and Dy: ', Mytmp)
        # for i in range(self.horiz_len - 2):
        #     Mytmp = np.hstack((Mytmp, np.zeros(MytmpShape)))
        # print('////////////////// 1st row of My: ', Mytmp)
        for i in range(1, self.horiz_len - 1):
            if Mytmp.ndim == 1:
                Mytmp = np.vstack((Mytmp, np.hstack((dot(self.Cy, \
                    dot(matpower(self.A, i), self.Bu )), Mytmp[:-self.nyBu+1]))))
            else:
                if dot(self.Cy, dot(matpower(self.A, i), self.Bu )).ndim == 1:
                    Mytmp = np.vstack(
                        (Mytmp,
                        np.hstack(
                            (dot(self.Cy, dot(matpower(self.A, i), self.Bu )),
                            Mytmp[-self.nxCy:,:-self.nyBu][0])
                        ))
                    )
                else:
                    Mytmp = np.vstack(
                        (Mytmp,
                        np.hstack(
                            (dot(self.Cy, dot(matpower(self.A, i), self.Bu )),
                            Mytmp[-self.nxCy:,:-self.nyBu])
                        ))
                    )
        self.My = Mytmp
        # print('************* My: ', self.My)

        Mztmp0 = self.Dzu
        Mztmp0Shape = np.shape(Mztmp0)
        Mztmp = np.hstack((dot(self.Cz, self.Bu), Mztmp0))
        for i in range(self.horiz_len - 1):
            Mztmp0 = np.hstack((Mztmp0, np.zeros(Mztmp0Shape)))
        # print('Mztmp0: ', Mztmp0)
        for i in range(self.horiz_len - 2):
            Mztmp = np.hstack((Mztmp, np.zeros(Mztmp0Shape)))
        # print('Mztmp: ', Mztmp)
        Mztmp = np.vstack((Mztmp0, Mztmp))
        # print('hhhhhhhhhhhhhhhh: ', Mztmp)

        for i in range(1, self.horiz_len - 1):
            if Mztmp.ndim == 1:
                Mztmp = np.vstack(
                    (Mztmp,
                    np.hstack(
                        (dot(self.Cz, dot(matpower(self.A, i), self.Bu )),
                        Mztmp[-self.nyBu+1:])
                    ))
                )
            else:
                if dot(self.Cz, dot(matpower(self.A, i), self.Bu )).ndim == 1:
                    Mztmp = np.vstack(
                        (Mztmp,
                        np.hstack(
                            (dot(self.Cz, dot(matpower(self.A, i), self.Bu )),
                            Mztmp[-self.nxCz:,:-self.nyBu][0])
                        ))
                    )
                else:
                    Mztmp = np.vstack(
                        (Mztmp,
                        np.hstack(
                            (dot(self.Cz, dot(matpower(self.A, i), self.Bu )),
                            Mztmp[-self.nxCz:,:-self.nyBu])
                        ))
                    )
        self.Mz = Mztmp
        # print('/////////// Mz: ', self.Mz)

        # TODO: need to update with Dyv
        Nytmp = dot(self.Cy, self.Bv)
        NytmpShape = np.shape(Nytmp)
        for i in range(self.horiz_len - 1):
            Nytmp = np.hstack((Nytmp, np.zeros(NytmpShape)))
        for i in range(1, self.horiz_len):
            if Nytmp.ndim == 1:
                Nytmp = np.vstack((Nytmp, np.hstack((dot(self.Cy, \
                    dot(matpower(self.A, i), self.Bv )), Nytmp[:-self.nyBv]))))
            else:
                if dot(self.Cy, dot(matpower(self.A, i), self.Bv )).ndim == 1:
                    Nytmp = np.vstack(
                        (Nytmp,
                        np.hstack(
                            (dot(self.Cy, dot(matpower(self.A, i), self.Bv )),
                            Nytmp[-self.nxCy:,:-self.nyBv][0])
                    ))
                )
                else:
                    Nytmp = np.vstack(
                        (Nytmp,
                        np.hstack(
                            (dot(self.Cy, dot(matpower(self.A, i), self.Bv )),
                            Nytmp[-self.nxCy:,:-self.nyBv])
                        ))
                    )
        self.Ny = Nytmp

        Nztmp = dot(self.Cz, self.Bv)
        NztmpShape = np.shape(Nztmp)
        for i in range(self.horiz_len - 1):
            Nztmp = np.hstack((Nztmp, np.zeros(NztmpShape)))
        Nztmp0 = np.zeros(np.shape(Nztmp))
        Nztmp = np.vstack((Nztmp0, Nztmp))
        for i in range(1, self.horiz_len - 1):
            if Nztmp.ndim == 1:
                Nztmp = np.vstack((Nztmp, np.hstack((dot(self.Cz, \
                    dot(matpower(self.A, i), self.Bv )), Nztmp[:-self.nyBv]))))
            else:
                if dot(self.Cz, dot(matpower(self.A, i), self.Bv )).ndim == 1:
                    Nztmp = np.vstack(
                        (Nztmp,
                        np.hstack(
                            (dot(self.Cz, dot(matpower(self.A, i), self.Bv )),
                            Nztmp[-self.nxCz:,:-self.nyBv][0])
                        ))
                    )
                else:
                    Nztmp = np.vstack(
                        (Nztmp,
                        np.hstack(
                            (dot(self.Cz, dot(matpower(self.A, i), self.Bv )),
                            Nztmp[-self.nxCz:,:-self.nyBv])
                        ))
                    )
        self.Nz = Nztmp


        Pytmp0 = self.Dyd
        Pytmp0Shape = np.shape(Pytmp0)
        Pytmp = np.hstack((dot(self.Cy, self.Bd), Pytmp0))
        for i in range(self.horiz_len - 1):
            Pytmp0 = np.hstack((Pytmp0, np.zeros(Pytmp0Shape)))
        for i in range(self.horiz_len - 2):
            Pytmp = np.hstack((Pytmp, np.zeros(Pytmp0Shape)))
        Pytmp = np.vstack((Pytmp0, Pytmp))

        # Pytmp = dot(self.Cy, self.Bd)
        # PytmpShape = np.shape(Pytmp)
        # for i in range(self.horiz_len - 1):
        #     Pytmp = np.hstack((Pytmp, np.zeros(PytmpShape)))

        for i in range(1, self.horiz_len - 1):
            if Pytmp.ndim == 1:
                Pytmp = np.vstack((Pytmp, np.hstack((dot(self.Cy, \
                    dot(matpower(self.A, i), self.Bd )), Pytmp[:-self.nyBd]))))
            else:
                if dot(self.Cy, dot(matpower(self.A, i), self.Bd )).ndim == 1:
                    Pytmp = np.vstack(
                        (Pytmp,
                        np.hstack(
                            (dot(self.Cy, dot(matpower(self.A, i), self.Bd )),
                            Pytmp[-self.nxCy:,:-self.nyBd][0])
                    ))
                )
                else:
                    Pytmp = np.vstack(
                        (Pytmp,
                        np.hstack(
                            (dot(self.Cy, dot(matpower(self.A, i), self.Bd )),
                            Pytmp[-self.nxCy:,:-self.nyBd])
                        ))
                    )
        self.Py = Pytmp


        Pztmp0 = self.Dzd
        Pztmp0Shape = np.shape(Pztmp0)
        Pztmp = np.hstack((dot(self.Cz, self.Bd), Pztmp0))
        for i in range(self.horiz_len - 1):
            Pztmp0 = np.hstack((Pztmp0, np.zeros(Pztmp0Shape)))
        for i in range(self.horiz_len - 2):
            Pztmp = np.hstack((Pztmp, np.zeros(Pztmp0Shape)))
        Pztmp = np.vstack((Pztmp0, Pztmp))

        # Pztmp = dot(self.Cz, self.Bd)
        # PztmpShape = np.shape(Pztmp)
        # for i in range(self.horiz_len - 1):
        #     Pztmp = np.hstack((Pztmp, np.zeros(PztmpShape)))
        # Pztmp0 = np.zeros(np.shape(Pztmp))
        # Pztmp = np.vstack((Pztmp0, Pztmp))

        for i in range(1, self.horiz_len - 1):
            if Pztmp.ndim == 1:
                Pztmp = np.vstack((Pztmp, np.hstack((dot(self.Cz, \
                    dot(matpower(self.A, i), self.Bd )), Pztmp[:-self.nyBd]))))
            else:
                if dot(self.Cz, dot(matpower(self.A, i), self.Bd )).ndim == 1:
                    Pztmp = np.vstack(
                        (Pztmp,
                        np.hstack(
                            (dot(self.Cz, dot(matpower(self.A, i), self.Bd )),
                            Pztmp[-self.nxCz:,:-self.nyBd][0])
                        ))
                    )
                else:
                    Pztmp = np.vstack(
                        (Pztmp,
                        np.hstack(
                            (dot(self.Cz, dot(matpower(self.A, i), self.Bd )),
                            Pztmp[-self.nxCz:,:-self.nyBd])
                        ))
                    )
        self.Pz = Pztmp

        if self.Fy.ndim == 1:
            self.nxFy = 1
            self.nyFy = 0
        else:
            self.nxFy = np.shape(self.Fy)[0]
            self.nyFy = np.shape(self.Fy)[1]

        if self.Fz.ndim == 1:
            self.nxFz = 1
            self.nyFz = 0
        else:
            self.nxFz = np.shape(self.Fz)[0]
            self.nyFz = np.shape(self.Fz)[1]

        if self.My.ndim == 1:
            self.nxMy = 1
            self.nyMy = 0
        else:
            self.nxMy = np.shape(self.My)[0]
            self.nyMy = np.shape(self.My)[1]

        if self.Mz.ndim == 1:
            self.nxMz = 1
            self.nyMz = 0
        else:
            self.nxMz = np.shape(self.Mz)[0]
            self.nyMz = np.shape(self.Mz)[1]

        if self.Ny.ndim == 1:
            self.nxNy = 1
            self.nyNy = 0
        else:
            self.nxNy = np.shape(self.Ny)[0]
            self.nyNy = np.shape(self.Ny)[1]

        if self.Nz.ndim == 1:
            self.nxNz = 1
            self.nyNz = 0
        else:
            self.nxNz = np.shape(self.Nz)[0]
            self.nyNz = np.shape(self.Nz)[1]

        if self.Py.ndim == 1:
            self.nxPy = 1
            self.nyPy = 0
        else:
            self.nxPy = np.shape(self.Py)[0]
            self.nyPy = np.shape(self.Py)[1]

        if self.Pz.ndim == 1:
            self.nxPz = 1
            self.nyPz = 0
        else:
            self.nxPz = np.shape(self.Pz)[0]
            self.nyPz = np.shape(self.Pz)[1]

        # TODO: make this work for user-supplied Q's and S's
        self.Q = np.diag(np.ones(self.nxMy)*1e3)
        # Set penalties for temperature to zero
        for i in np.arange(0, len(self.Q), 2):
            self.Q[i] = np.zeros(self.nxMy)

        # Set penalty for last power to zero
        self.Q[-1] = np.zeros(self.nxMy)
        # self.Q[3::2] = np.zeros(self.nxMy)

        self.S = np.diag(np.zeros(self.nyMy)*1.0e-3)

    def mat_sizes(self):
        if self.A.ndim == 1:
            self.nxA = 1
            self.nyA = 1
        elif self.A.ndim == 2:
            self.nxA = np.shape(self.A)[0]
            self.nyA = np.shape(self.A)[1]
        else:
            raise Exception("The 'A' matrix has > 2 dimensions")

        if self.Bu.ndim == 1:
            self.nxBu = 1
            self.nyBu = 1
        elif self.Bu.ndim == 2:
            self.nxBu = np.shape(self.Bu)[0]
            self.nyBu = np.shape(self.Bu)[1]
        else:
            raise Exception("The 'Bu' matrix has > 2 dimensions")

        if self.Bv.ndim == 1:
            self.nxBv = 1
            self.nyBv = 1
        elif self.Bv.ndim == 2:
            self.nxBv = np.shape(self.Bv)[0]
            self.nyBv = np.shape(self.Bv)[1]
        else:
            raise Exception("The 'Bv' matrix has > 2 dimensions")

        if self.Bd.ndim == 1:
            self.nxBd = 1
            self.nyBd = 1
        elif self.Bd.ndim == 2:
            self.nxBd = np.shape(self.Bd)[0]
            self.nyBd = np.shape(self.Bd)[1]
        else:
            raise Exception("The 'Bd' matrix has > 2 dimensions")

        if self.Cy.ndim == 1:
            self.nxCy = 1
            self.nyCy = 1
        elif self.Cy.ndim == 2:
            self.nxCy = np.shape(self.Cy)[0]
            self.nyCy = np.shape(self.Cy)[1]
        else:
            raise Exception("The 'Cy' matrix has > 2 dimensions")

        if self.Cz.ndim == 1:
            self.nxCz = 1
            self.nyCz = 1
        elif self.Cz.ndim == 2:
            self.nxCz = np.shape(self.Cz)[0]
            self.nyCz = np.shape(self.Cz)[1]
        else:
            raise Exception("The 'Cz' matrix has > 2 dimensions")

        if self.Dyu.ndim == 1:
            self.nxDyu = 1
            self.nyDyu = 1
        elif self.Dyu.ndim == 2:
            self.nxDyu = np.shape(self.Dyu)[0]
            self.nyDyu = np.shape(self.Dyu)[1]
        else:
            raise Exception("The 'Dy' matrix has > 2 dimensions")

        if self.Dyv.ndim == 1:
            self.nxDyv = 1
            self.nyDyv = 1
        elif self.Dyv.ndim == 2:
            self.nxDyv = np.shape(self.Dyv)[0]
            self.nyDyv = np.shape(self.Dyv)[1]
        else:
            raise Exception("The 'Dy' matrix has > 2 dimensions")

        # if self.Dz.ndim == 1:
        #     self.nxDz = 1
        #     self.nyDz = 0
        # elif self.Dz.ndim == 2:
        #     self.nxDz = np.shape(self.Dz)[0]
        #     self.nyDz = np.shape(self.Dz)[1]
        # else:
        #     raise Exception("The 'Dz' matrix has > 2 dimensions")
