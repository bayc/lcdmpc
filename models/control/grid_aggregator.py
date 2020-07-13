import numpy as np
import autograd.numpy as np  # thinly-wrapped numpy
from numpy import dot as dot
from numpy import transpose as tp
from autograd import grad    # the only autograd function you may ever need

class grid_aggregator:
    def __init__(self, horiz_len, num_downstream):

        self.horiz_len = horiz_len
        self.num_downstream = num_downstream

        inputs = np.array([0.0])
        disturbances = np.array([0.0])
        self.reinit(inputs, disturbances)

    def reinit(self, inputs, disturbances):
        # inputs

        # disturbances
        # self.disturbances = disturbances

        # Model matrices
        self.A = np.zeros((self.num_downstream, self.num_downstream))
        self.Bu = np.zeros((self.num_downstream, self.num_downstream))
        self.Bv = np.eye(self.num_downstream)
        self.Bd = np.zeros((self.num_downstream, 1)) 
        
        self.Bu_mean_inputs = np.zeros((self.num_downstream, self.num_downstream))
        self.Bd_mean_inputs = np.array([0.0])
        self.Cy_mean_outputs = np.array([0.0])
        self.Cz_mean_outputs = np.zeros((self.num_downstream, 1))

        self.Cy = np.array([[1.0]*self.num_downstream])
        self.Dyu = np.array([[0.0]*self.num_downstream])
        self.Dyv = np.array([[0.0]])
        self.Dyd = np.array([[0.0]])

        self.Cz = np.zeros((self.num_downstream, self.num_downstream))
        self.Dzu = np.eye(self.num_downstream)
        self.Dzv = np.zeros((self.num_downstream, self.num_downstream))
        self.Dzd = np.zeros((self.num_downstream, 1))

        self.Cy_lin = np.array([0.0])
        self.Cz_lin = np.array([0.0])

        self.Dyu_lin = np.array([0.0])
        self.Dzu_lin = np.array([0.0])

        self.Dyd_lin = np.array([0.0])
        self.Dzd_lin = np.array([0.0])

    def get_forecast(self, current_time, disturbance_data):
        return np.array([0.0])

    def parse_opt_vars(self, varDict):
        self.bldg_Prefs = varDict['bldg_Prefs']

        return self.bldg_Prefs

    def parse_sol_vars(self, sol):
        self.bldg_Prefs = list(sol.getDVs().values())[0]

        return self.bldg_Prefs

    def add_var_group(self, optProb):
        optProb.addVarGroup('bldg_Prefs', self.horiz_len*self.num_upstream, type='c', 
                            lower=0.0, upper=100.0, value=35.0)

        self.numDVs = len(optProb.getDVs().keys())
        return optProb

    def add_con_group(self, optProb):
        return optProb

    def compute_cons(self, funcs, uOpt, Y):
        return funcs

    def update_inputs(self, states, control_actions, outputs):
        return np.array([0.0])

    def update_disturbances(self, d):
        self.d = np.array([0.0])

    def process_refs(self, refs):
        return refs

    def process_refs_horiz(self, refs, refs_const):
        return refs

    def sensitivity_func(self):
        return 0.0