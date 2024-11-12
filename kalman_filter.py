

import numpy as np



# Part 3: Comment the code explaining each part
class kalman_filter:
    
    # Part 3: Initialize the covariances and the states    
    def __init__(self, P,Q,R, x, dt):
        
        self.P = P
        self.Q = Q
        self.R = R
        self.x = x
        self.dt = dt
        
    # Part 3: Replace the matrices with Jacobians where needed        
    def predict(self):

        self.A = self.jacobian_A()
        self.C = self.jacobian_H()
        
        self.motion_model()
        
        self.P= np.dot( np.dot(self.A, self.P), self.A.T) + self.Q

    # Part 3: Replace the matrices with Jacobians where needed
    def update(self, z):

        S=np.dot(np.dot(self.C, self.P), self.C.T) + self.R
            
        kalman_gain=np.dot(np.dot(self.P, self.C.T), np.linalg.inv(S))
        
        surprise_error= z - self.measurement_model()
        
        self.x=self.x + np.dot(kalman_gain, surprise_error)
        self.P=np.dot( (np.eye(self.A.shape[0]) - np.dot(kalman_gain, self.C)) , self.P)
        
    
    # Part 3: Implement here the measurement model
    def measurement_model(self):
        # Don't need x, y, or th when unpacking
        _, _, _, w, v, vdot = self.x

        return np.array([
            v, # v - linear velocity along the forward (x) direction
            w, # w - angular velocity
            vdot, # ax - acceleration in the forward direction
            v * w, # ay - lateral acceleration
        ])
        
    # Part 3: Implement the motion model (state-transition matrix)
    def motion_model(self):
        x, y, th, w, v, vdot = self.x
        dt = self.dt
        
        # vdot and w are constant over the timestep
        self.x = np.array([
            x + v * np.cos(th) * dt,
            y + v * np.sin(th) * dt,
            th + w * dt,
            w,
            v + vdot*dt,
            vdot
        ])

    def jacobian_A(self):
        # Don't need x, y, w, vdot when unpacking
        _, _, th, _, v, _ = self.x
        dt = self.dt
        
        # 
        return np.array([
            #x, y,               th, w,               v, vdot
            [1, 0,              -v * np.sin(th) * dt, 0, np.cos(th) * dt,  0],
            [0, 1,              v * np.cos(th) * dt, 0,  np.sin(th) * dt,  0],
            [0, 0,                1, dt,           0,  0],
            [0, 0,                0, 1,            0,  0],
            [0, 0,                0, 0,            1,  dt],
            [0, 0,                0, 0,            0,  1 ]
        ])
    
    
    # Part 3: Implement here the jacobian of the H matrix (measurements)    
    def jacobian_H(self):
        # Dont need x, y, th, or vdot when unpacking
        _, _, _, w, v, _ = self.x

        # partial(ay) with respect to w is v
        # partial(ay) with respect to v is w
        return np.array([
            #x, y,th, w, v,vdot
            [0,0,0  , 0, 1, 0], # v
            [0,0,0  , 1, 0, 0], # w
            [0,0,0  , 0, 0, 1], # ax
            [0,0,0  , v, w, 0], # ay
        ])
        
    # Part 3: return the states here    
    def get_states(self):
        x, y, th, w, v, vdot = self.x
        return np.array([x, y, th, w, v, vdot])
