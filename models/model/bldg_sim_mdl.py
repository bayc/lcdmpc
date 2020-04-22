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
        self.P = np.array([[]]) # covariance matrix of the building states
        self.R = 0.1 # variance in room temperature measurement
    
    def bldg_sim_filter_update(self,n_steps = 6, EP_data):
        # Run filter update at start of each day for n_filter_steps to match initial 
        #conditions with E+ simulation
        if not isinstance(args, tuple):
            args = (args,)
        if not isinstance(hx_args, tuple):
            hx_args = (hx_args,)

        if R is None:
            R = self.R

        if np.isscalar(z) and self.dim_z == 1:
            z = np.asarray([z], float)
            
     # Calculating Kalman filter gain
        H = HJacobian(self.x, *args)
        PHT = dot(self.P, H.T)
        self.S = dot(H, PHT) + R
        self.K = PHT.dot(np.linalg.inv(self.S))
        hx = Hx(self.x, *hx_args)
        if n_pred > 1:
            self.x_pred = copy.deepcopy(self.x)
            self.res_pred = residual(z, hx)
            for i in range(n_pred -1):
                EP_data = self.get_data(EP_sim_data_pd2,k + i)
                u = EP_data[1:]
                self.x_pred = self.move(self.x_pred,u,self.dt)
                if self.x_pred[0][0] > 12 and self.x_pred[0][0]< 35:
                    self.res_pred += abs(residual(EP_sim_data_pd2.T_room[k+i+1], self.x_pred[0][0]))
                else:
                    self.res_pred += 15
        self.res = residual(z, hx)
        if n_pred > 1:
            self.x[0] += dot(self.K[0], self.res)
            self.x[1:] += dot(self.K[1:],self.res_pred)
        else:
            self.x = self.x + dot(self.K, self.res)

        # P = (I-KH)P(I-KH)' + KRK' is more numerically stable
        # and works for non-optimal K vs the equation
        # P = (I-KH)P usually seen in the literature.
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