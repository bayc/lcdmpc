import numpy as np
import autograd.numpy as np  # thinly-wrapped numpy
from numpy import dot as dot
from numpy import transpose as tp
from autograd import grad    # the only autograd function you may ever need

class bldg61_control:
    def __init__(self, ms_dot, T_sa, T_z, horiz_len, energy_red_weight,
    Qint_std, Qsol_std, Qint_scale, Qsol_scale, Qint_offset, Qsol_offset):
        # TODO: normalize to make automatic
        self.gamma_scale = 1e0

        # fan power (kW) model coefficients
        self.a0 = -0.0444
        self.a1 = -0.0317
        self.a2 = 0.1474
        self.hvac_cop = 3.274

        self.horiz_len = horiz_len

        # keys to use to grab distubance values from input .csv
        self.disturb_keys = ['T_outside', 'Q_internal', 'Q_solar']

        # intial values for model
        self.truth_model_T_z = np.array(T_z)
        self.truth_model_Pwr = np.array(20.)
        inputs = [ms_dot, T_sa, T_z]
        disturbances = np.zeros(len(self.disturb_keys))
        self.reinit(inputs, disturbances)

        # self.Z_idn = [0]

        # parameters for arbitrarily adjusting input values to create variation
        self.Qint_std = Qint_std
        self.Qsol_std = Qsol_std
        self.Qint_scale = Qint_scale
        self.Qsol_scale = Qsol_scale
        self.Qint_offset = Qint_offset
        self.Qsol_offset = Qsol_offset

        self.energy_red_weight = energy_red_weight

    def reinit(self, inputs, disturbances):
        # inputs
        ms_dot = inputs[0]
        T_sa = inputs[1]
        T_z = inputs[2]

        # disturbances
        T_oa = disturbances[0]
        Q_int = disturbances[1]
        Q_solar = disturbances[2]

        # Model matrices
        # self.A = np.array([[0.9745]]) # 5 minute model
        self.A = np.array([[0.9961]]) # 1 minute model
        # coefficients for Q_hvac (-ve value for cooling) - self.mean_inputs(4)
        # self.Bu = np.array([[0.00716, 0.0, 0.0]])  # 5 minute model
        self.Bu = np.array([[0.00133, 0.0, 0.0]]) # 1 minute model
        self.Bv = np.array([[0.0]])
        # self.Bd = np.array([[0.001, 0.00799, 0.00332]])  # 5 minute data
        self.Bd = np.array([[0.000131, 0.001354, 0.0006067]]) # 1 minute model

        # filter parameter
        self.K = 0.9961

        # mean inputs used when model was identified
        self.Bu_mean_inputs = np.array([0.0, 0.0, 0.0])
        self.Bd_mean_inputs = np.array([[21.23], [0.0], [0.0]])
        self.Cy_mean_outputs = np.array([
            [21.02],
            [self.truth_model_Pwr],
            [self.truth_model_Pwr]
        ])
        self.Cz_mean_outputs = np.array([self.truth_model_Pwr])

        # compute building power model parameters (Cz, Dzu, Dzv, Dzd, linearized values)
        self.bldg_power_model(
            ms_dot,
            T_sa,
            T_oa - self.Bd_mean_inputs[0][0],
            T_z - self.Cy_mean_outputs[0][0]
        )

        # output equation model matrices
        # Building outputs are 1) zone temperature, 2) building power, and
        # 3) error/difference between building power and grid aggregator power reference
        self.Cy = np.array([[1.], self.Cz[0], -1*self.Cz[0]])
        self.Dyu = np.array([[0.0, 0.0, 0.0], self.Dzu[0], -1*self.Dzu[0]])
        self.Dyv = np.array([[0.0], self.Dzv[0], [1.0 - self.Dzv[0]]])
        self.Dyd = np.array([[0.0, 0.0, 0.0], self.Dzd[0], -1*self.Dzd[0]])

        # y and z equation linearization values
        self.Cy_lin = dot(
            np.array([[0.], [self.Cy[1]], [-1*self.Cy[1]]]), self.T_z_lin
        )
        self.Cz_lin = dot(self.Cz[0], self.T_z_lin)

        self.Dyu_lin = dot(
            np.array([np.zeros(np.shape(self.Dyu)[1]).tolist(), self.Dyu[1], -1*self.Dyu[1]]), 
            np.array([[0.], [self.ms_dot_lin], [self.T_sa_lin]])
        )
        self.Dzu_lin = dot(
            self.Dzu, 
            np.array([[0.], [self.ms_dot_lin], [self.T_sa_lin]])
        )

        self.Dyd_lin = dot(
            self.Dyd,
            np.array([[self.T_oa_lin], [0.], [0.]])
        )
        self.Dzd_lin = dot(
            self.Dzd, 
            np.array([[self.T_oa_lin], [0.], [0.]])
        )

    def process_Q(self, Q):
        # Set penalties for temperature ref. tracking
        for i in np.arange(0, len(Q), 3):
            if i == 0:
                Q[i] = np.zeros(len(Q))
            else:
                Q[i] = Q[i]*1.0e0 # 5.0e1, 2.0, -2; 0 for combined opt
        # Set penalties for absolute power ref tracking to zero
        for i in np.arange(1, len(Q), 3):
            Q[i] = np.zeros(len(Q))
        for i in np.arange(2, len(Q), 3):
            # Q[i] = np.zeros(len(Q))
            Q[i] = Q[i]*1.0e3 # 3.0e0, 2.0

        # manual scaling of weight parameters
        # TODO: normalize to automate weighting
        # return Q*1.0e-5
        return Q

    def process_S(self, S):
        # manual scaling of weight parameters
        # TODO: normalize to automate weighting
        # S = S*5.0e-8
        S = S*0.0
        # S = S*self.energy_red_weight
        return S

    def filter_update(self, states, outputs):
        T_z_truth = outputs[0]
        states = states + self.K * (T_z_truth - self.Cy_mean_outputs[0][0] - dot(self.Cy[0], states))
        return states

    def process_refs(self, refs):
        refs = refs - np.array([0, self.truth_model_Pwr, self.truth_model_Pwr]) \
            + self.Cy_lin \
            + self.Dyu_lin \
            + self.Dyd_lin
        return refs

    def process_refs_horiz(self, refs, refs_const):
        refs = (np.array(refs_const) \
            - np.array([0, self.truth_model_Pwr, -1*self.truth_model_Pwr]*self.horiz_len).reshape(np.shape(refs_const)[0], 1) \
            + np.array([self.Cy_lin]*self.horiz_len).reshape(np.shape(refs_const)[0], 1) \
            + np.array([self.Dyu_lin]*self.horiz_len).reshape(np.shape(refs_const)[0], 1) \
            + np.array([self.Dyd_lin]*self.horiz_len).reshape(np.shape(refs_const)[0], 1))

        return refs

    def get_forecast(self, current_time, disturbance_data):
        # read in next set of forecast information
        tmp = disturbance_data.iloc[current_time:current_time + self.horiz_len]
        vars = []
        for key in self.disturb_keys:
            if key is 'Q_internal':
                vars.append(tmp[key].values*self.Qint_scale + self.Qint_offset)
            elif key is 'Q_solar':
                vars.append(tmp[key].values*self.Qsol_scale + self.Qsol_offset)
            else:
                offset = 0.0
                vars.append(tmp[key].values)

        return np.array([val for tup in zip(*vars) for val in tup]).reshape(np.shape(vars)[0]*np.shape(vars)[1], 1)

    def parse_opt_vars(self, varDict):
        self.Qhvac = varDict['Qhvac']
        self.ms_dot = varDict['ms_dot']
        self.T_sa = varDict['T_sa']

        vars = [self.Qhvac, self.ms_dot, self.T_sa]
        return np.array([val for tup in zip(*vars) for val in tup]).reshape(np.shape(vars)[0]*np.shape(vars)[1], 1)

    def parse_sol_vars(self, sol):
        self.Qhvac = list(sol.getDVs().values())[0]
        self.ms_dot = list(sol.getDVs().values())[1]
        self.T_sa = list(sol.getDVs().values())[2]

        vars = [self.Qhvac, self.ms_dot, self.T_sa]
        return np.array([val for tup in zip(*vars) for val in tup]).reshape(np.shape(vars)[0]*np.shape(vars)[1], 1)

    def add_var_group(self, optProb):
        optProb.addVarGroup('Qhvac', self.horiz_len, type='c', 
                            lower=-1200.0, upper=0.0, value=-32.)
        optProb.addVarGroup('ms_dot', self.horiz_len, type='c', 
                            lower=0.0, upper=20.0, value=0.8)
        optProb.addVarGroup('T_sa', self.horiz_len, type='c', 
                            lower=10.0, upper=14.0, value=12.8)

        self.numDVs = len(optProb.getDVs().keys())
        return optProb

    def add_con_group(self, optProb):
        optProb.addConGroup('hvac_con', self.horiz_len, lower=0, upper=0)
        optProb.addConGroup(
            'T_building_con', self.horiz_len, lower=20.0, upper=23.3
        )

        return optProb

    def compute_cons(self, funcs, uOpt, Y):
        # Add constraints for the SNOPT optimization
        Qhvac = uOpt[0::3]
        ms_dot = uOpt[1::3]
        T_sa = uOpt[2::3]

        funcs['hvac_con'] = (np.array(Qhvac) - np.array(ms_dot)*(np.array(T_sa) \
                        - (self.truth_model_T_z))).reshape(self.horiz_len)
        funcs['T_building_con'] = np.array(Y[0::3]).reshape(self.horiz_len)

        return funcs

    def bldg_fan_power(self, inputs):
        ms_dot = inputs[1]
        # P_fan = self.a0*ms_dot**3 + self.a1*ms_dot**2 + self.a2*ms_dot \
        #       + self.a3
        P_fan = self.a0 + self.a1*ms_dot + self.a2*ms_dot**2
        return P_fan

    def bldg_chiller_power(self, inputs):
        ms_dot = inputs[1]
        T_sa = inputs[0]
        T_oa = inputs[2] + self.Bd_mean_inputs[0][0]
        T_z = inputs[3] + self.Cy_mean_outputs[0][0]
        T_ma = 0.3*T_oa + (1 - 0.3)*T_z
        P_chill = 1.005/self.hvac_cop*ms_dot*(T_ma - T_sa)
        return P_chill

    def bldg_power_model(self, ms_dot, T_sa, T_oa, T_z):
        grad_fan = grad(self.bldg_fan_power)
        grad_chill = grad(self.bldg_chiller_power)

        self.ms_dot_lin = ms_dot
        self.T_sa_lin = T_sa
        self.T_oa_lin = T_oa
        self.T_z_lin = T_z

        D_fan = grad_fan([T_sa, ms_dot, T_oa , T_z])
        D_chill = grad_chill([T_sa, ms_dot, T_oa, T_z])

        Dz = np.array(D_fan) + np.array(D_chill)
        self.Cz = np.array([[Dz[3]]])
        self.Dzu = np.array([np.hstack(([0.0], Dz[1], Dz[0]))])
        self.Dzv = np.array([[0.0]])
        self.Dzd = np.array([np.hstack((Dz[2], [0.0, 0.0]))])

    def update_inputs(self, states, control_actions, outputs):
        ms_dot = control_actions[1]
        T_sa = control_actions[2]
        self.truth_model_T_z = outputs[0][0][0]
        self.truth_model_Pwr = outputs[1][0][0]
        return [ms_dot, T_sa, self.truth_model_T_z]

    def update_disturbances(self, d):
        T_oa = d[0]
        Q_int = d[1]
        Q_solar = d[2]
        self.d = [T_oa, Q_int, Q_solar]

    def process_uConv(self, uConv):
        uConv[0::3] = np.array(uConv[1::3])*(np.array(uConv[2::3]) \
                        - (self.truth_model_T_z))
        return np.array(uConv).reshape(np.shape(uConv)[0], 1)
