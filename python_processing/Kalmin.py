import numpy as np

ndim = 4
dt = 0.75  # sec

class Kalman:
    def __init__(self, x_init, cov_init, meas_err_slam, meas_err_odom, proc_err):
        self.ndim = len(x_init)
        self.A = np.array([(1, 0, dt, 0), (0, 1, 0, dt), (0, 0, 1, 0), (0, 0, 0, 1)])
        self.H_ultrasound = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])  # Observation matrix for ultrasound (y-coordinate only)
        self.H_rf = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])  # Observation matrix for RF (x and y coordinates)
        self.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])  # Observation matrix for RF (x and y coordinates)
        self.x_hat =  x_init
        self.cov = cov_init
        self.Q_k = np.eye(self.ndim) * proc_err
        self.R_ultrasound = np.eye(len(self.H_ultrasound)) * meas_err_slam
        self.R_rf = np.eye(len(self.H_rf)) * meas_err_odom

    def update_slam(self, obs):
        #Make prediction
        self.x_hat_est = np.dot(self.A, self.x_hat)
        self.cov_est = np.dot(self.A, np.dot(self.cov, np.transpose(self.A))) + self.Q_k


        # Update estimate
        self.error_x = obs - np.dot(self.H,self.x_hat_est)
        self.error_cov = np.dot(self.H,np.dot(self.cov_est,np.transpose(self.H))) + self.R_ultrasound
        self.K = np.dot(np.dot(self.cov_est,np.transpose(self.H)),np.linalg.inv(self.error_cov))
        self.x_hat = self.x_hat_est + np.dot(self.K,self.error_x)
        if ndim > 1:
            self.cov = np.dot((np.eye(self.ndim) - np.dot(self.K,self.H)),self.cov_est)
        else:
            self.cov = (1-self.K)*self.cov_est 

    def update_odom(self, obs):
        # Update estimate using RF measurements

        self.x_hat_est = np.dot(self.A, self.x_hat)
        self.cov_est = np.dot(self.A, np.dot(self.cov, np.transpose(self.A))) + self.Q_k


        # Update estimate
        self.error_x = obs - np.dot(self.H,self.x_hat_est)
        self.error_cov = np.dot(self.H,np.dot(self.cov_est,np.transpose(self.H))) + self.R_rf
        self.K = np.dot(np.dot(self.cov_est,np.transpose(self.H)),np.linalg.inv(self.error_cov))
        self.x_hat = self.x_hat_est + np.dot(self.K,self.error_x)
        if ndim>1:
            self.cov = np.dot((np.eye(self.ndim) - np.dot(self.K,self.H)),self.cov_est)
        else:
            self.cov = (1-self.K)*self.cov_est
