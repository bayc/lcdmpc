# M. Sinner 8/22/19
import pandas as pd
import numpy as np

def extract_admittance_matrix(csv_filename):
    """
    Get admittance matrix produced by `Export Y' in openDSS.

    Inputs:
        csv_filename - str - filename of .CSV file produced by `Export Y'
    
    Outputs:
        Y - NxN array of complex - complex nodal admittance matrix
    """

    df_Y_sep = pd.read_csv(csv_filename, header=None, skiprows=1)
    
    df_Y_sep.drop(df_Y_sep.columns[-1], axis=1, inplace=True)
    df_Y_sep.set_index(0, inplace=True)

    # Convert (separated) Y matrix to numeric
    for c in range(len(df_Y_sep.columns[:])):
        for r in range(len(df_Y_sep)):
            if str(df_Y_sep.iloc[r, c])[0:3] == ' +j':
                df_Y_sep.iloc[r, c] = float(df_Y_sep.iloc[r, c][4:])*1j
            else:
                df_Y_sep.iloc[r, c] = float(df_Y_sep.iloc[r, c])

    # Convert to (complex) numpy array
    Y_separated = df_Y_sep.to_numpy()
    real_cols = np.arange(0,np.shape(Y_separated)[1],2)
    imag_cols = np.arange(1,np.shape(Y_separated)[1],2)
    Y = Y_separated[:, real_cols] + Y_separated[:, imag_cols]
    if ~(Y == Y.T).all():
        raise TypeError('Y not symmetric!')
    
    return Y

def single_phase_equivalent(admittance_matrix_3phase):
    """
    Convert balanced 3-phase system to single phase equivalent.
    
    Inputs:
        admittance_matrix_3phase - (3(N+1))x(3(N+1)) array of complex

    Outputs:
        (admittance_matrix_1phase) - (N+1)x(N+1) array of complex
    """

    # Take only the A phase entries
    keep_rows = np.arange(0, np.shape(admittance_matrix_3phase)[0], 3)
    
    return admittance_matrix_3phase[keep_rows,:][:,keep_rows]

def linear_power_flow_constraint(admittance_matrix, s_lin, v_lin, v_0=1+0j):
    """
    Construct inverse linearized PFE of form v' = As' + b.
    
    Inputs:
        admittance_matrix - (N+1)x(N+1) array of complex - system 
                                                           admittance
                                                           matrix
        s_lin - Nx1 array of complex - nodal complex powers to linearize
                                       about (at `load' nodes)
        v_lin - Nx1 array of complex - nodal voltage to linearize about
                                       (at `load' nodes)
        v_0 - complex - voltage at the slack bus (defaults to per unit)
    Outputs:
        A - 2Nx2N array of real - constraint system matrix
        b - 2Nx1 array of real - constraint affine term
    """ 

    Jac_s = construct_power_flow_Jacobian(admittance_matrix, v_lin, v_0)
    
    A = np.linalg.inv(Jac_s)

    b = -A @ s_lin + v_lin

    return A, b

def construct_power_flow_Jacobian(admittance_matrix, v_lin, v_0=1+0j):
    """
    Get Jacobian for use in linearized PFE for load nodes.
    
    Inputs:
        admittance_matrix - (N+1)x(N+1) array of complex - system 
                                                           admittance
                                                           matrix
        v_lin - Nx1 array of complex - nodal voltage to linearize about
                                       (at `load' nodes)
        v_0 - complex - voltage at the slack bus (defaults to per unit)
    Outputs:
        Jac_s - 2Nx2N array of real - system Jacobian matrix
    """

    # construct matrices, vectors in real form
    Y_LL_prime = _build_Y_prime(admittance_matrix[1:,1:])
    Y_L0_prime = _build_Y_prime(admittance_matrix[1:,0].reshape(-1,1))
    v_0_prime = np.array([[np.real(v_0)], [np.image(v_0)]])
    v_lin_prime = np.concatenate([np.real(v_lin), np.imag(v_lin)])
    
    # Calculate power flow mismatch Jacobian
    Jac_f_v = power_flow_mismatch_Jacobian(Y_LL_prime,
                                           Y_L0_prime,
                                           v_lin_prime,
                                           v_0_prime)

    # Power flow Jacobian is additive inverse    
    Jac_s = -Jac_f_v
    
    return Jac_s

def power_flow_mismatch_Jacobian(Y_LL_prime, Y_L0_prime, v_prime, v_0_prime):
    """
    Find Jacobian (w.r.t v') of power flow mismatch equation.

    Inputs:
        Y_LL_prime - 2Nx2N array of real - load bus admittance matrix
        Y_L0_prime - 2Nx2 array of real - slack to load bus admittance 
        v_prime - 2Nx1 array of real - load voltages
        v_0_prime - 2x1 array of real - slack voltages
    
    Outputs:
        Jac - 2Nx2N array of real - system Jacobian matrix
    """
    N = int(round(np.shape(Y_LL_prime)[0]/2))
    Jac = np.zeros([2*N, 2*N])
    for k in range(N):
        for m in range(N):
            if m == k:
                kronecker_delta = 1
            else:
                kronecker_delta = 0
            
            # Top left (R,R)
            Jac[k,m] =  - (v_prime[k][0] * Y_LL_prime[k,m] + \
                           v_prime[N+k][0] * Y_LL_prime[N+k,m]) \
                        - kronecker_delta * (Y_LL_prime[k,:] @ v_prime + \
                                             Y_L0_prime[k,:] @ v_0_prime)
                       
            
            # Top right (R,I)
            Jac[k,N+m] = - (v_prime[N+k][0] * Y_LL_prime[N+k,N+m] + \
                            v_prime[k][0] * Y_LL_prime[k,N+m]) \
                          - kronecker_delta * (Y_LL_prime[N+k,:] @ v_prime + \
                                               Y_L0_prime[N+k,:] @ v_0_prime)
                          
            
            # Bottomr left (I,R)
            Jac[N+k,m] = - (v_prime[N+k][0] * Y_LL_prime[k,m] - \
                            v_prime[k][0] * Y_LL_prime[N+k,m]) \
                          + kronecker_delta * (Y_LL_prime[N+k,:] @ v_prime + \
                                               Y_L0_prime[N+k,:] @ v_0_prime)
                          
            
            # Bottom right (I,I)
            Jac[N+k,N+m] = - (v_prime[N+k][0] * Y_LL_prime[k,N+m] - \
                              v_prime[k][0] * Y_LL_prime[N+k,N+m]) \
                           - kronecker_delta * (Y_LL_prime[k,:] @ v_prime +
                                                Y_L0_prime[k,:] @ v_0_prime)

    return Jac

def convert_to_per_unit(admittance_matrix, nominal_power, nominal_voltage):
    """
    Convert admittance matrix to `per unit' units.

    Inputs:
        admittance_matrix - (N+1)x(N+1) array of complex - system 
                                                           admittance
                                                           matrix
        nominal_power - float - nominal real power of the system
        nominal_voltage - float - nominal real voltage of the system
        
    Outputs:
        admittance_matrix_pu - (N+1)x(N+1) array of complex - system 
                                                              admittance
                                                              matrix in
                                                              per unit
                                                              form
        nominal_properties - dict - nominal properties of system
    """

    # Assign passed-in values
    nominal_properties = {
        'power' : nominal_power,
        'voltage' : nominal_voltage
    }
        
    # Find other nominal values
    nominal_properties['current'] = nominal_power/(np.sqrt(3)*nominal_voltage)
    nominal_impedance = nominal_voltage/ \
                        (np.sqrt(3)*nominal_properties['current'])
    nominal_properties['admittance'] = 1/nominal_impedance

    # Convert admittance matrix
    admittance_matrix_pu = admittance_matrix/nominal_properties['admittance']

    return admittance_matrix_pu, nominal_properties

def _build_Y_prime(admittance_matrix):
    """ 
    Construct the admittance matrix for the real form.

    Inputs:
        admittance_matrix - (N+1)x(N+1) array of complex - system 
                                                           admittance
                                                           matrix
        
    Outputs:
        Y_prime - (2(N+1) x 2(N+1)) array - real form of admittance 
                                            matrix
    """
    Y_prime = np.append(np.append(np.real(admittance_matrix), 
                                  -np.imag(admittance_matrix), 
                                  1),
                        np.append(np.imag(admittance_matrix),
                                  np.real(admittance_matrix),
                                  1),
                        0)
    return Y_prime
