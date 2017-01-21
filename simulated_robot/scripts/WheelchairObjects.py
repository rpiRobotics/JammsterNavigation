import numpy as np
import math
import copy
import scipy.integrate as integrate
from scipy import linalg

# Adapted from http://greg.czerniak.info/guides/kalman1/
class KalmanFilterLinear:
    def __init__(self,_A, _B, _H, _x, _P, _Q, _R):
        self.A = _A                      # State transition matrix.
        self.B = _B                      # Control matrix.
        self.H = _H                      # Observation matrix.
        self.current_state_estimate = _x # Initial state estimate.
        self.current_prob_estimate = _P  # Initial covariance estimate.
        self.Q = _Q                      # Estimated error in process.
        self.R = _R                      # Estimated error in measurements.
        
    def get_current_state(self):
        return self.current_state_estimate
        
    def update(self, measurement_vector):
        """ Update Kalman filter if sensing information is received """
       #--------------------------Observation step-----------------------------
        innovation = measurement_vector - np.dot(self.H, self.current_state_estimate)
        innovation_covariance = np.dot(np.dot(self.H, self.current_prob_estimate), np.transpose(self.H)) + self.R
        #-----------------------------Update step-------------------------------
        kalman_gain = np.dot(np.dot(self.current_prob_estimate, np.transpose(self.H)), np.linalg.inv(innovation_covariance))
        self.current_state_estimate = self.current_state_estimate + np.dot(kalman_gain, innovation)
        # We need the size of the matrix so we can make an identity matrix.
        size = self.current_prob_estimate.shape[0]
        # eye(n) = nxn identity matrix.
        self.current_prob_estimate = np.dot((np.eye(size)-np.dot(kalman_gain,self.H)), self.current_prob_estimate)
        
    def predict(self, control_vector):
        """ Step without sensing """
        self.current_state_estimate = np.dot(self.A, self.current_state_estimate) + np.dot(self.B, control_vector)
        self.current_prob_estimate = np.dot(np.dot(self.A, self.current_prob_estimate), np.transpose(self.A)) + self.Q

class Motor:
    """ Base class for a motor object, is implemented to be a simulated motor
        Extend for real motor"""
    def __init__(self, starting_state, track_history = True):
        self.track_history = track_history
        self.dt = .01

        self.k1 = -1        # motor constant
        self.k2 = 1            # motor constant
        
        self.saturation = 40     # absolute saturation velocity
        
        self.t_last_m = 0
        self.u = 0
        
        self.desired_speed = 0
        self.state_history = []
        self.control_history = []
                
        # Discrete state space
        self.A = math.exp(self.k1 * self.dt)
        self.B = (1.0/self.k1)*(self.A - 1)*self.k2
        self.C = np.eye(2)
        self.Q = np.eye(2)*.1  # variance of process noise
        self.R = np.eye(2)      # variance of sensing noise
        
        self.kf = KalmanFilterLinear(self.A, self.B, self.C, starting_state, np.zeros([2,2]), self.Q, self.R)
        
        # Control gains
        self.K = np.ones([1,2])
        self.K[0,0] = 0
        self.K[0,1] = 1
        
        self.set_desired_speed(0)
        
    def set_desired_speed(self, desired_speed):
        """ Set desired speed for the motor, calculate steady state current 
            with feed forward"""
        self.set_point = desired_speed
        self.feed_forward_u = -self.k1*desired_speed/self.k2        
    
    def update(self, time_now):
        """ Update state estimate and control
            1) Update kfilter prediction
            2) OPTIONALLY incorporate measurement
            2) Update control """
  
        self.dt = -(self.t_last_m - time_now)      
        self.t_last_m = time_now         
        
        ### UPDATE KFILTER PARAMS ###
        self.A = math.exp(self.k1 * self.dt)
        self.B = (1.0/self.k1)*(self.A - 1)*self.k2
        self.kf.A = self.A
        self.kf.B = self.B
                
        self.kf.predict(self.u)
        
        ## OPTIONALLY incorporate measurement ###
        measurement = self._obtain_measurement()
        if not measurement==[]: 
            self.kf.update(measurement)
       
        ### UPDATE CONTROL ###
        self._update_control()


    def update_dt(self, dt):
        """ Update state estimate and control
            1) Update kfilter prediction
            2) OPTIONALLY incorporate measurement
            2) Update control """
  
        self.dt = dt
        self.t_last_m += dt         
        
        ### UPDATE KFILTER PARAMS ###
        self.A = math.exp(self.k1 * self.dt)
        self.B = (1.0/self.k1)*(self.A - 1)*self.k2
        self.kf.A = self.A
        self.kf.B = self.B
                
        self.kf.predict(self.u)
        
        ## OPTIONALLY incorporate measurement ###
        measurement = self._obtain_measurement()
        if not measurement==[]: 
            self.kf.update(measurement)
       
        ### UPDATE CONTROL ###
        self._update_control()

    def _update_control(self):
        """ find new control using feedback """
        self.u = np.dot(self.K, self.set_point-self.state) + self.feed_forward_u
        if self.u > self.saturation:
            self.u = self.saturation
            
        if self.u < -self.saturation:
            self.u = -self.saturation
            
        self._set_voltage()
        
    def _set_voltage(self):
        """ IMPLEMENT ME FOR SPECIFIC MOTOR"""
        return
        
    def _obtain_measurement(self):
        """ IMPLEMENT ME FOR SPECIFIC MOTOR """
        return []

class SimulatedMotor(Motor):
    def __init__(self, starting_state, track_history = True):
        self.track_history = track_history
        self.dt = .1
        
        self.k1 = 2.3364e-04          # motor constant
        self.k2 = .65            # motor constant
        self.state = starting_state
        self.saturation = 40     # absolute saturation velocity
        self.Q = 0.00001 # variance of process noise
        self.R = np.eye(2)*.001   # variance of sensing noise
        
        self.t_last_m = 0
        self.u = 0
        
        self.desired_speed = 0
        self.state_history = []
        self.control_history = []
                
        # Discrete state space
        self.A = math.exp(self.k1 * self.dt)
        self.B = (1.0/self.k1)*(self.A - 1)*self.k2
        self.C = np.ones([2,1])
        
        self.kf = KalmanFilterLinear(self.A, self.B, self.C, self.state, np.zeros([1,1]), self.Q, self.R)
        self.K = np.ones([1,1])
        self.K = 1
        
        self.feed_forward_error_param = 1 # scalar multiple of how off the feed forward model is from true required control
                
        
        self.set_desired_speed(0)        
        self.num_sim_measurements = 0
        
    def set_desired_speed(self, desired_speed):
        """ Set desired speed for the motor, calculate steady state current 
            with feed forward"""
        self.set_point = desired_speed
        self.feed_forward_u = -self.k1*desired_speed*self.feed_forward_error_param/self.k2   
        
    def _obtain_measurement(self):
        """ WILL NOT BE PRESENT IN NON-SIMULATED"""
        # keep track of measurements provided so far
        self.num_sim_measurements += 1
        self._step_simulated_system() 
        
        return np.array([self.state + np.random.normal(0, self.R[0,0])])
        
    def _step_simulated_system(self):
        """ apply a voltage to the motor for specified time, simulated has to update true state """
        # UPDATE REAL STATE
        self.state = np.dot(self.A, self.state) + np.dot(self.B, self.u) + np.random.normal(0, self.Q)
        if self.track_history:
            self.state_history.append(self.state)
            self.control_history.append(self.u)