import numpy as np


class KF():
    def __init__(self, sigma_a, s):

        # x(+1) = x + dt*u
        dt = 1
        self.F = np.array([[1, 0, 0, dt, 0, 0],
                           [0, 1, 0, 0, dt, 0],
                           [0, 0, 1, 0, 0, dt],
                           [0, 0, 0, 1, 0, 0],
                           [0, 0, 0, 0, 1, 0],
                           [0, 0, 0, 0, 0, 1]])
        # x(+1) = ... + dt^2 * a
        self.B = np.array([[dt**2, 0, 0],
                           [0, dt**2, 0],
                           [0, 0, dt**2],
                           [dt, 0, 0],
                           [0, dt, 0],
                           [0, 0, dt]])
        self.x = np.zeros(6)
        self.P = np.diag([1, 1, 1, 1, 1, 1])
        return

    def predict(self, a, dt):
        self.F[0, 3] = dt
        self.F[1, 4] = dt
        self.F[2, 5] = dt

        self.B[0, 0] = dt**2
        self.B[1, 1] = dt**2
        self.B[2, 2] = dt**2
        self.B[3, 0] = dt
        self.B[4, 1] = dt
        self.B[5, 2] = dt

        self.x = np.dot(self.F, self.x) + np.dot(self.B, a)
        self.P = np.dot(self.F, np.dot(self.P, self.F.T)) + self.Q
        return self.x


    '''
    def update(self, z):
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        y = z - np.dot(self.H, self.x)
        self.x = self.x + np.dot(K, y)
        I = np.eye(self.P.shape[0])
        self.P = np.dot(I - np.dot(K, self.H), self.P)
        return self.x
        '''
