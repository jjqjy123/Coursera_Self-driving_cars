import numpy as np
import matplotlib.pyplot as plt

class Vehicle():
    def __init__(self):
 
        # ==================================
        #  Parameters
        # ==================================
    
        #Throttle to engine torque
        self.a_0 = 400
        self.a_1 = 0.1
        self.a_2 = -0.0002
        
        # Gear ratio, effective radius, mass + inertia
        self.GR = 0.35
        self.r_e = 0.3
        self.J_e = 10
        self.m = 2000
        self.g = 9.81
        
        # Aerodynamic and friction coefficients
        self.c_a = 1.36
        self.c_r1 = 0.01
        
        # Tire force 
        self.c = 10000
        self.F_max = 10000
        
        # State variables
        self.x = 0
        self.v = 5
        self.a = 0
        self.w_e = 100
        self.w_e_dot = 0
        
        self.sample_time = 0.01
        
    def reset(self):
        # reset state variables
        self.x = 0
        self.v = 5
        self.a = 0
        self.w_e = 100
        self.w_e_dot = 0

    def step(self, throttle, alpha):
        # control input
        w_w = self.GR*self.w_e
        s = (w_w*self.r_e - self.v)/self.v

        if abs(s)<1:
            F_x = self.c*s
        else:
            F_x = self.F_max

        F_aero = self.c_a*self.v**2
        R_x = self.c_r1*self.v
        F_g = self.m*self.g*np.sin(alpha)
        F_load = F_aero + R_x + F_g

        self.a = (F_x - F_load)/self.m       
        self.x += self.v*self.sample_time
        self.v += self.a*self.sample_time
        
        T_e = throttle*(self.a_0 + self.a_1*self.w_e + self.a_2*self.w_e**2)
        self.w_e_dot = (T_e - self.GR*self.r_e*F_load)/self.J_e
        self.w_e += self.w_e_dot*self.sample_time 

def flat():
    sample_time = 0.01
    time_end = 100
    model = Vehicle()

    t_data = np.arange(0,time_end,sample_time)
    v_data = np.zeros_like(t_data)

    # throttle percentage between 0 and 1
    throttle = 0.2

    # incline angle (in radians)
    alpha = 0

    for i in range(t_data.shape[0]):
        v_data[i] = model.v
        model.step(throttle, alpha)
        
    plt.plot(t_data, v_data)
    plt.show()

def slope():
    time_end = 20
    sample_time = 0.01
    t_data = np.arange(0,time_end,sample_time)
    x_data = np.zeros_like(t_data)
    model = Vehicle()

    throttle_data = np.zeros_like(t_data)
    alpha_data = np.zeros_like(t_data) 
    for i in range(t_data.shape[0]):
        if i<500:
            throttle_data[i] = 0.06*i*sample_time + 0.2
        elif i<1500:
             throttle_data[i] = 0.5
        else:
            throttle_data[i] = -0.1*i*sample_time + 2

    for i in range(t_data.shape[0]):
        if model.x < 60:
            alpha_data[i] = np.arctan(3/60)       
        elif model.x < 150:          
            alpha_data[i] = np.arctan(9/90)
        else:           
            alpha_data[i] = 0
        
        model.step(throttle_data[i], alpha_data[i])
        x_data[i] = model.x

    plt.plot(t_data, x_data)
    plt.show()
    
#flat()
slope()