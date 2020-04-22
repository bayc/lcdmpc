import numpy as np

class bldg_sim_mdl:
    def __init__(self, dt, ms_dot, T_sa, Q_internal,Q_solar, T_oa, T_r,T_e):
    # bldg fan power coefficients
        self.a0 = 0.0029
        self.a1 = -0.0151
        self.a2 = 0.1403
        self.a3 = 0.0086
        
        self.hvac_cop = 3 # hvac cop
        
        # initial values of room air and wall temperatures
        self.T_r = T_r
        self.T_e = T_e
        self.x = np.array([[T_r,T_e]]) # states for Kalman filter
        
        self.dt = 1/12 # timestep in hrs
        self.i_sim = 0 # current simulation time index
        # inputs = mass flow rate, supply air temp, internal heat (kW), solar gain (kW)
        # , and outside air temperature 
        self.inputs = [ms_dot, T_sa, Q_internal, Q_solar, T_oa]
        self.reinit()

    def reinit(self):

        # 3R2C model parameters (inverse of the values)
        self.C_r_inv = np.array([[0.01626433]]) # room air capacitance inverse
        self.R_re_inv = np.array([[8.845245]]) # room air to wall resistance inverse
        self.R_ra_inv = np.array([[3.42398]]) # room air to outside air resistance inverse
        self.C_e_inv = np.array([[0.0009977]]) # external wall equivalent cpaacitance inverse
        self.R_ea_inv = np.array([[22.2297201]]) # external wal to outside air resistance inverse
        
        # Uncertainty matrices
        self.P = np.array([[2,0],[0,2]]) # covariance matrix of the building states
        self.R = 0.1 # variance in room temperature measurement
        
    def HJacobian_at(x):
    """ compute Jacobian of H matrix at x """
    return np.array ([[1.,0.]])

    def Hx(x):
    """ compute output as a function of states
    """
    return x[0][0]
    
    def bldg_sim_filter_update(self,EP_data,n_steps = 6, R=None, \
                               args=(), hx_args=(), residual=np.subtract):
        # Run filter update at start of each day for n_filter_steps to match initial 
        #conditions with E+ simulation
        
        # arguments for Jacobian of H matrix 
        if not isinstance(args, tuple):
            args = (args,)
        # arguments for hx
        if not isinstance(hx_args, tuple):
            hx_args = (hx_args,)
        # output measurement coviariance
        if R is None:
            R = self.R
            
        # apply filter update n_steps times
        for i in range(n_steps):
            EP_inputs = EP_data[self.i_sim,1:]
            T_r_EP = EP_data[self.i_sim,:][0]
            z = np.asarray([z], float) # output value for Kalman filter update
            
     # Calculating Kalman filter gain
            H = self.HJacobian(self.x, *args)
            PHT = dot(self.P, H.T)
            self.S = dot(H, PHT) + R
            self.K = PHT.dot(np.linalg.inv(self.S))
            hx = self.Hx(self.x, *hx_args)
     # Measurement update applied to states
            self.x = self.x + dot(self.K, self.res)
            self.T_r = self.x[0][0]
            self.T_e = self.x[1][0]
            
    # update covariance matrix
            I_KH = self._I - dot(self.K, H)
            self.P = dot(I_KH, self.P).dot(I_KH.T) + dot(self.K, R).dot(self.K.T)
        
    def bldg_simulation_step(self):
        # all inputs to the 3R2C model
        dt = self.dt
        ms_dot = self.inputs[0]
        T_sa = self.inputs[1]
        Q_hvac = ms_dot*(T_sa - self.T_r)
        Q_internal = self.inputs[2]
        Q_solar = self.inputs[3]
        T_oa = inputs[4]
        
        # 3R2C room temperature dynamics
        dTr = dt*(self.C_r_inv*self.R_re_inv*(self.T_e - self.T_r) +\ # change in room temperature
              self.C_r_inv*self.R_ra_inv*(T_oa - self.T_r) + \
              self.C_r_inv*(Q_solar + Q_internal + Q_hvac))

        dTe = dt*(self.C_e_inv*self.R_re_inv*(self.T_r - self.T_e) + \ # change in wall temperature
              self.C_e_inv*self.R_ea_inv*(T_oa - self.T_e) + \
              self.C_e_inv*(Q_solar + Q_internal + Q_hvac))
    
        # update room and wall temperatres
        self.T_r += dTr
        self.T_e += dTe
        
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

        self.bldg_power_model(T_sa, ms_dot, T_oa + self.Bd_mean_inputs[0], T_z + self.Cy_mean_outputs)