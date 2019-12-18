import numpy as np
import autograd.numpy as np  # thinly-wrapped numpy
from autograd import grad    # the only autograd function you may ever need

class bldg_data_driven_mdl:
    def __init__(self, ms_dot, T_sa, T_oa, T_z, horiz_len):
        # fan power (kW) model coefficients
        self.a0 = 0.0029
        self.a1 = -0.0151
        self.a2 = 0.1403
        self.a3 = 0.0086
        self.hvac_cop = 3

        self.horiz_len = horiz_len

        inputs = [ms_dot, T_sa, T_oa, T_z]
        self.reinit(inputs)

    def reinit(self, inputs):
        ms_dot = inputs[0]
        T_sa = inputs[1]
        T_oa = inputs[2]
        T_z = inputs[3]

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
        self.Bu = np.array([[0.0008, 0.0, 0.0]]) # coefficients correspondig to Q_hvac (-ve value for cooling) - self.mean_inputs(4)
        self.Bv = np.array([[0.0]])
        self.Bd = np.array([[0.001, 0.0017, 0.0016]]) #[T_out , Q_int, Q_solar] - self.mean_inputs(1:3)
        self.Cy = np.array([[1.]])
        self.Dyu = np.array([[0.0, 0.0, 0.0]])
        self.Dyv = np.array([[0.0]])
        self.Dyd = np.array([[0.0, 0.0, 0.0]])
        self.K = 0.8826
        self.Bu_mean_inputs = np.array([0.0, 0.0, 0.0])
        self.Bd_mean_inputs = np.array([19.96, 0.0, 0.0])
        self.Cy_mean_outputs = 23.56

        self.bldg_power_model(T_sa, ms_dot, T_oa + self.Bd_mean_inputs[0], T_z + self.Cy_mean_outputs)

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
        return optProb

    def add_con_group(self, optProb):
        optProb.addConGroup('hvac_con', self.horiz_len, lower=0, upper=0)
        optProb.addConGroup('T_building_con', self.horiz_len, lower=-2, upper=2)

        return optProb

    def compute_cons(self, funcs, uOpt, Y):
        # Add constraints for the SNOPT optimization
        Qhvac = uOpt[0::3]
        ms_dot = uOpt[1::3]
        T_sa = uOpt[2::3]

        funcs['hvac_con'] = np.array(Qhvac) - np.array(ms_dot)*(np.array(T_sa) \
                        - (Y + 23.56))
        funcs['T_building_con'] = Y

        return funcs

    def bldg_fan_power(self, inputs):
        ms_dot = inputs[0]
        P_fan = self.a0*ms_dot**3 + self.a1*ms_dot**2 + self.a0*ms_dot \
              + self.a3
        return P_fan

    def bldg_chiller_power(self, inputs):
        ms_dot = inputs[0]
        T_sa = inputs[1]
        T_oa = inputs[2]
        self.T_oa = T_oa
        T_z = inputs[3]
        T_ma = 0.3*T_oa + (1 - 0.3)*T_z
        P_chill = 1.005/self.hvac_cop*ms_dot*(T_ma - T_sa)
        return P_chill
        
    def bldg_power_model(self, ms_dot, T_sa, T_oa, T_z):
        grad_fan = grad(self.bldg_fan_power)
        grad_chill = grad(self.bldg_chiller_power)

        D_fan = grad_fan([ms_dot, T_sa, T_oa, T_z])
        D_chill = grad_chill([ms_dot, T_sa, T_oa, T_z])

        Dz = np.array(D_fan) + np.array(D_chill)
        self.Cz = np.array([[Dz[3]]])
        self.Dzu = np.array([np.hstack(([0.0], Dz[0:2]))])
        self.Dzv = np.array([[0.0]])
        self.Dzd = np.array([np.hstack((Dz[2], [0.0, 0.0]))])