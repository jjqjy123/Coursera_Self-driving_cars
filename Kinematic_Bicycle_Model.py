import numpy as np
import matplotlib.pyplot as plt

class Bicycle():                                                  #: create Model of Bicycle
    def __init__(self):
        self.xc = 0
        self.yc = 0
        self.theta = 0
        self.delta = 0
        self.beta = 0
        
        self.L = 2                                                #: wheelbase length
        self.lr = 1.2                                             #: length between center of mass and the rear axle
        self.w_max = 1.22                                         #: max turning rate
        
        self.sample_time = 0.01                                   #: sampling time, fps
        
    def reset(self):
        self.xc = 0
        self.yc = 0
        self.theta = 0
        self.delta = 0
        self.beta = 0

    def step(self, v, w):
        if w>0:
            w=min(w, self.w_max)
        if w<0:
            w=max(w, -self.w_max)
            
        T = 0.01
        
        xc_dot = v*np.cos(self.theta + self.beta)                 #: based of equations of bicycle kinematics
        yc_dot = v*np.sin(self.theta + self.beta)
        theta_dot = v*np.cos(self.beta)*np.tan(self.delta)/self.L
        delta_dot = w
        
        self.xc += xc_dot*T
        self.yc += yc_dot*T
        self.theta += theta_dot*T
        self.delta += delta_dot*T
        self.beta = np.arctan(self.lr*np.tan(self.delta)/self.L)

def path_circle():
    sample_time = 0.01
    time_end = 20
    model = Bicycle()

    t_data = np.arange(0,time_end,sample_time)
    x_data = np.zeros_like(t_data)
    y_data = np.zeros_like(t_data)

    for i in range(t_data.shape[0]):
        x_data[i] = model.xc
        y_data[i] = model.yc
        if model.delta < np.arctan(2/10):                              #: the steering angle cannot be directly set
            model.step(np.pi, model.w_max)
        else:
            model.step(np.pi, 0)
        # model.beta = 0
    
    plt.axis('equal')
    plt.plot(x_data, y_data,label='Learner Model')
    plt.legend()
    plt.show()

def path_square():
    sample_time = 0.01
    time_end = 60
    model = Bicycle()

    t_data = np.arange(0,time_end,sample_time)
    x_data = np.zeros_like(t_data)                                     #: create the same size matrix with 0
    y_data = np.zeros_like(t_data)

    # maintain velocity at 4 m/s
    v_data = np.zeros_like(t_data)
    v_data[:] = 4 

    w_data = np.zeros_like(t_data)
    # ==================================
    #  Square Path: set w at corners only
    # ==================================
    w_data[670:670+100] = 0.753
    w_data[670+100:670+100*2] = -0.753
    w_data[2210:2210+100] = 0.753
    w_data[2210+100:2210+100*2] = -0.753
    w_data[3670:3670+100] = 0.753
    w_data[3670+100:3670+100*2] = -0.753
    w_data[5220:5220+100] = 0.753
    w_data[5220+100:5220+100*2] = -0.753
    for i in range(t_data.shape[0]):
        x_data[i] = model.xc
        y_data[i] = model.yc
        model.step(v_data[i], w_data[i])
    
    plt.axis('equal')
    plt.plot(x_data, y_data,label='Learner Model')
    plt.legend()
    plt.show()

def path_spiral():
    sample_time = 0.01
    time_end = 60
    model = Bicycle()

    t_data = np.arange(0,time_end,sample_time)
    x_data = np.zeros_like(t_data)
    y_data = np.zeros_like(t_data)

    # maintain velocity at 4 m/s
    v_data = np.zeros_like(t_data)
    v_data[:] = 4 

    w_data = np.zeros_like(t_data)
    # ==================================
    #  Spiral Path: high positive w, then small negative w
    # ==================================
    w_data[:] = -1/100
    w_data[0:100] = 1

    for i in range(t_data.shape[0]):                                          #: .shape[0] calculate 1st dimension of matrix
        x_data[i] = model.xc
        y_data[i] = model.yc
        model.step(v_data[i], w_data[i])
    
    plt.axis('equal')
    plt.plot(x_data, y_data,label='Learner Model')
    plt.legend()
    plt.show()

def path_wave():
    sample_time = 0.01
    time_end = 60
    model = Bicycle()

    t_data = np.arange(0,time_end,sample_time)
    x_data = np.zeros_like(t_data)
    y_data = np.zeros_like(t_data)

    # maintain velocity at 4 m/s
    v_data = np.zeros_like(t_data)
    v_data[:] = 4 

    w_data = np.zeros_like(t_data)
    # ==================================
    #  Wave Path: square wave w input
    # ==================================
    w_data[:] = 0
    w_data[0:100] = 1
    w_data[100:300] = -1
    w_data[300:500] = 1
    w_data[500:5700] = np.tile(w_data[100:500], 13)                            #: enlarge the range of data with same data in[100:500]
    w_data[5700:] = -1

    for i in range(t_data.shape[0]):
        x_data[i] = model.xc
        y_data[i] = model.yc
        model.step(v_data[i], w_data[i])
    
    plt.axis('equal')
    plt.plot(x_data, y_data,label='Learner Model')
    plt.legend()
    plt.show()

def path_8():
    sample_time = 0.01
    time_end = 30
    model = Bicycle()

    t_data = np.arange(0,time_end,sample_time)
    x_data = np.zeros_like(t_data)
    y_data = np.zeros_like(t_data)
    v_data = np.zeros_like(t_data)
    w_data = np.zeros_like(t_data)

    R = 8
    v_data[:] = 2*2*np.pi*R/time_end     
    delta = 0.98*np.arctan(2/8)
    t1 = 375
    t2 = 1875
    for i in range(0, t1):
        x_data[i] = model.xc
        y_data[i] = model.yc
        if model.delta < delta:
            w_data[i] = model.w_max        
        else:
            w_data[i] = 0
        model.step(v_data[i], w_data[i])

    for i in range(t1, t2):
        x_data[i] = model.xc
        y_data[i] = model.yc
        if model.delta > -delta:
            w_data[i] = -model.w_max        
        else:
            w_data[i] = 0
        model.step(v_data[i], w_data[i])
        
    for i in range(t2, 3000):
        x_data[i] = model.xc
        y_data[i] = model.yc
        if model.delta < delta:
            w_data[i] = model.w_max/2        
        else:
            w_data[i] = 0
        model.step(v_data[i], w_data[i])

    plt.axis('equal')                                                                #: same scale of x and y axis
    plt.plot(x_data, y_data)
    plt.show()

#path_circle()
#path_square()
#path_spiral()
#path_wave()
path_8()




