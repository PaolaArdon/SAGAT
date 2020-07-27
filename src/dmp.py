import numpy as np

class CanonicalSystem():

    def __init__(self, dt, ax=1.0):

        self.ax = ax
        self.dt = dt
        self.timesteps = int(1.0/self.dt)
        self.reset_state()

    def rollout(self):

        timesteps = self.timesteps
        self.x_t = np.zeros(timesteps)

        self.reset_state()
        for t in range(timesteps):
            self.x_t[t] = self.x
            self.step()

        return self.x_t

    def step(self):
        self.x += (-self.ax * self.x)*self.dt
        return self.x

    def reset_state(self):
        self.x = 1.0

class DMP():

    def __init__(self,goal,y0, Nb=5, dt=0.05, d=2,jnames=[]):

        self.ay = np.ones(d)*25
        self.by = self.ay/4.
        self.dt = dt
        self.Nb = Nb
        self.d = d
        self.joint_names = jnames
        self.cs = CanonicalSystem(dt=self.dt)
        self.T = self.cs.timesteps

        des_c = np.linspace(0, 1, self.Nb)
        self.c = np.ones(len(des_c))
        for n in range(len(des_c)):
            self.c[n] = np.exp(-self.cs.ax * des_c[n])

        self.h = np.ones(self.Nb)*self.Nb**1.5 /self.c / self.cs.ax

        self.goal = goal
        self.y0 = y0

        self.y = self.y0.copy()
        self.dy = np.zeros(self.d)
        self.ddy = np.zeros(self.d)

    def rbf(self,x):
        return np.exp(-self.h * (x[:,None] - self.c)**2)

    def step(self):

        # step canonical system
        x = self.cs.step()

        # generate basis function activation
        psi = self.rbf(np.array([x]))

        for d in range(self.d):
            # generate the forcing term
            f = (x*(self.goal[d]-self.y0[d])*(np.dot(psi, self.w[d]))/np.sum(psi))

            # DMP acceleration
            self.ddy[d] = (self.ay[d]*(self.by[d]*(self.goal[d] - self.y[d]) - self.dy[d]) + f)

            self.dy[d] += self.ddy[d]*self.dt
            self.y[d] += self.dy[d]*self.dt

        return self.y, self.dy, self.ddy

    def rollout(self):

        self.reset_state()
        # set up tracking vectors
        y_track = np.zeros((self.T, self.d))
        dy_track = np.zeros((self.T, self.d))
        ddy_track = np.zeros((self.T, self.d))

        for t in range(self.T):
            # run and record timestep
            y_track[t], dy_track[t], ddy_track[t] = self.step()

        return y_track, dy_track, ddy_track

    def fit(yd,goal,t):

        d_yd = np.gradient(yd,0.1)
        dd_yd = np.gradient(d_yd,0.1)

        f_d = dd_yd - self.alpha_y*(self.beta_y*(goal-yd) - d_yd)

        phi = rbf(t)

    def imitate_path(self, y_d):

        # set initial state and goal
        self.y0 = y_d[0, :].copy()
        self.y_d = y_d.copy()
        self.goal = y_d[-1,:].copy()

        # generate function to interpolate the desired trajectory
        import scipy.interpolate
        path = np.zeros((self.d, self.T))
        x = np.linspace(0, 1, y_d.shape[0])
        for d in range(self.d):
            path_gen = scipy.interpolate.interp1d(x, y_d[:,d])
            for t in range(self.T):
                path[d, t] = path_gen(t*self.dt)
        y_d = path

        # calculate velocity of y_des
        dy_d = np.diff(y_d)/self.dt
        # add zero to the beginning of every row
        dy_d = np.hstack((np.zeros((self.d, 1)), dy_d))

        # calculate acceleration of y_des
        ddy_d = np.diff(dy_d)/self.dt
        # add zero to the beginning of every row
        ddy_d = np.hstack((np.zeros((self.d, 1)), ddy_d))

        f_target = np.zeros((y_d.shape[1], self.d))
        # find the force required to move along this trajectory
        for d in range(self.d):
            f_target[:, d] = (ddy_d[d] - self.ay[d]*(self.by[d]*(self.goal[d] - y_d[d]) - dy_d[d]))

        # efficiently generate weights to realize f_target
        self.gen_weights(f_target)

        return y_d

    def gen_weights(self, f_target):

        # calculate x and psi
        x_track = self.cs.rollout()
        psi_track = self.rbf(x_track)

        # efficiently calculate BF weights using weighted linear regression
        self.w = np.zeros((self.d, self.Nb))
        for d in range(self.d):
            # spatial scaling term
            k = (self.goal[d] - self.y0[d])
            for b in range(self.Nb):
                self.w[d, b] = (np.sum(x_track * psi_track[:, b]*f_target[:, d]))/(np.sum(x_track**2*psi_track[:, b])*k)
        self.w = np.nan_to_num(self.w)

        self.reset_state()

    def reset_state(self):
        self.y = self.y0.copy()
        self.dy = np.zeros(self.d)
        self.ddy = np.zeros(self.d)
        self.cs.reset_state()
