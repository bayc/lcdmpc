import numpy as np
import autograd.numpy as np  # thinly-wrapped numpy
from numpy import dot as dot
from numpy import transpose as tp
from autograd import grad    # the only autograd function you may ever need

class bldg_data_driven_mdl:
    def __init__(self, ms_dot, T_sa, T_z, horiz_len):
        # fan power (kW) model coefficients
        self.a0 = 0.0029
        self.a1 = -0.0151
        self.a2 = 0.1403
        self.a3 = 0.0086
        self.hvac_cop = 3

        self.horiz_len = horiz_len

        self.disturb_keys = ['T_outside', 'Q_internal', 'Q_solar']

        self.truth_model_T_z = np.array(T_z)
        self.truth_model_Pwr = np.array(20.)
        inputs = [ms_dot, T_sa, T_z]
        # disturbances = [T_oa, Q_int, Q_solar]
        disturbances = np.zeros(len(self.disturb_keys))
        self.reinit(inputs, disturbances)

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
        # self.A = np.array([[0.8826]])
        # self.Bu = np.array([[0.0014, 0.0, 0.0]]) # coefficients correspondig to Q_hvac (-ve value for cooling) - self.mean_inputs(4)
        # self.Bv = np.array([[0.0]])
        # self.Bd = np.array([[0.0116, 0.0024, 0.001]]) #[T_out , Q_int, Q_solar] - self.mean_inputs(1:3)
        # self.Cy = np.array([[1.]])
        # self.Dyu = np.array([[0.0, 0.0, 0.0]])
        # self.Dyv = np.array([[0.0]])
        # self.Dyd = np.array([[0.0, 0.0, 0.0]])
        self.A = np.array([[0.9931]])
        # coefficients correspondig to Q_hvac (-ve value for cooling) - self.mean_inputs(4)
        self.Bu = np.array([[0.0001263, 0.0, 0.0]]) 
        self.Bv = np.array([[0.0]])
        #[T_out , Q_int, Q_solar] - self.mean_inputs(0:2)
        self.Bd = np.array([[0.0004864, 0.0003279, 0.0003766]]) 
        
        self.K = 0.9998
        self.Bu_mean_inputs = np.array([0.0, 0.0, 0.0])
        self.Bd_mean_inputs = np.array([26.3525, 0.0, 0.0])
        self.Cy_mean_outputs = [22.794, self.truth_model_Pwr]
        self.Cz_mean_outputs = np.array([self.truth_model_Pwr])
        # self.Cy_mean_outputs = [22.794, 22.794]

        # self.d = [Q_int, Q_solar, T_oa]

        self.bldg_power_model(
            ms_dot,
            T_sa,
            T_oa - self.Bd_mean_inputs[0],
            T_z - self.Cy_mean_outputs[0]
        )

        self.Cy = np.array([[1.], self.Cz[0]])
        self.Dyu = np.array([[0.0, 0.0, 0.0], self.Dzu[0]])
        self.Dyv = np.array([[0.0], self.Dzv[0]])
        self.Dyd = np.array([[0.0, 0.0, 0.0], self.Dzd[0]])

        self.Cy_lin = dot(
            np.array([0., self.Cy[1]]), self.T_z_lin
        )
        self.Cz_lin = dot(self.Cz[0], self.T_z_lin)

        self.Dyu_lin = dot(
            np.array([np.zeros(np.shape(self.Dyu)[1]).tolist(), self.Dyu[1]]), tp(
                np.array(
                    [0., self.ms_dot_lin, self.T_sa_lin]
                )
            )
        )
        self.Dzu_lin = dot(self.Dzu, 
            tp(
                np.array(
                    [0., self.ms_dot_lin, self.T_sa_lin]
                )
            )
        )

        self.Dyd_lin = dot(
            self.Dyd,
            tp(
                np.array(
                    [self.T_oa_lin, 0., 0.]
                )
            )
        )
        self.Dzd_lin = dot(self.Dzd, 
            tp(
                np.array(
                    [self.T_oa_lin, 0., 0.]
                )
            )
        )

    def filter_update(self, states, outputs):
        T_z_truth = outputs[0]
        print('pre-states: ', states + self.Cy_mean_outputs[0])
        states = states + self.K * (T_z_truth - self.Cy_mean_outputs[0] - dot(self.Cy[0], states))
        print('post-states: ', states + self.Cy_mean_outputs[0])
        return states[0]

    def get_forecast(self, current_time, disturbance_data):
        # read in next set of forecast information
        tmp = disturbance_data.iloc[current_time:current_time + self.horiz_len]
        vars = []
        for key in self.disturb_keys:
            vars.append(tmp[key].values)

        return [val for tup in zip(*vars) for val in tup]

    def parse_opt_vars(self, varDict):
        self.Qhvac = varDict['Qhvac']
        self.ms_dot = varDict['ms_dot']
        self.T_sa = varDict['T_sa']

        vars = [self.Qhvac, self.ms_dot, self.T_sa]
        return [val for tup in zip(*vars) for val in tup]

    def parse_sol_vars(self, sol):
        self.Qhvac = list(sol.getDVs().values())[0]
        self.ms_dot = list(sol.getDVs().values())[1]
        self.T_sa = list(sol.getDVs().values())[2]

        vars = [self.Qhvac, self.ms_dot, self.T_sa]
        return [val for tup in zip(*vars) for val in tup]

    def add_var_group(self, optProb):
        optProb.addVarGroup('Qhvac', self.horiz_len, type='c', 
                            lower=-200.0, upper=0.0, value=-100.)
        optProb.addVarGroup('ms_dot', self.horiz_len, type='c', 
                            lower=0.0, upper=20.0, value=0.8)
        optProb.addVarGroup('T_sa', self.horiz_len, type='c', 
                            lower=10.0, upper=14.0, value=12.8)

        self.numDVs = len(optProb.getDVs().keys())
        return optProb

    def add_con_group(self, optProb):
        optProb.addConGroup('hvac_con', self.horiz_len, lower=0, upper=0)
        optProb.addConGroup(
            'T_building_con', self.horiz_len, lower=21, upper=25
        )

        return optProb

    def compute_cons(self, funcs, uOpt, Y):
        # Add constraints for the SNOPT optimization
        Qhvac = uOpt[0::3]
        ms_dot = uOpt[1::3]
        T_sa = uOpt[2::3]

        funcs['hvac_con'] = np.array(Qhvac) - np.array(ms_dot)*(np.array(T_sa) \
                        - (self.truth_model_T_z))
        funcs['T_building_con'] = Y[0::2]
        # print('funcs[T_building_con]: ', funcs['T_building_con'])

        return funcs

    def bldg_fan_power(self, inputs):
        ms_dot = inputs[1]
        P_fan = self.a0*ms_dot**3 + self.a1*ms_dot**2 + self.a2*ms_dot \
              + self.a3
        # P_fan = ms_dot
        print('control P_fan: ', P_fan)
        return P_fan
        # return 0.0

    def bldg_chiller_power(self, inputs):
        print('control chiller power inputs: ', inputs)
        ms_dot = inputs[1]
        T_sa = inputs[0]
        T_oa = inputs[2] + self.Bd_mean_inputs[0]
        # self.T_oa = T_oa
        T_z = inputs[3] + self.Cy_mean_outputs[0]
        T_ma = 0.3*T_oa + (1 - 0.3)*T_z
        P_chill = 1.005/self.hvac_cop*ms_dot*(T_ma - T_sa)
        # P_chill = ms_dot + T_z
        print('control P_chill: ', P_chill)
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

        print('D_fan grad: ', D_fan)
        print('D_chill grad: ', D_chill)

        Dz = np.array(D_fan) + np.array(D_chill)
        print('Dz grad: ', Dz)
        self.Cz = np.array([[Dz[3]]])
        self.Dzu = np.array([np.hstack(([0.0], Dz[1], Dz[0]))])
        self.Dzv = np.array([[0.0]])
        self.Dzd = np.array([np.hstack((Dz[2], [0.0, 0.0]))])

    def update_inputs(self, states, control_actions, outputs):
        # TODO: need to define
        # Tie last measurement to T_z input
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