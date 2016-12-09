
import numpy as np

class Particle(object):
    """A particle in n-dimensions driven by forces."""

    def __init__(self, x0=[0,0], v0=[0,0], inv_mass=1.):
        self.x = np.asarray(x0, dtype=float)        
        self.xprev = np.asarray(self.x - np.asarray(v0), dtype=float)
        self.inv_mass = inv_mass
        self.fnet = np.zeros(self.x.shape)

    def add_force(self, f):
        self.fnet += f

    @property
    def v(self):
        return self.x - self.xprev

    @property
    def mass(self):
        return 1.0 / self.inv_mass

    def update(self, dt):

        a = self.fnet * self.inv_mass

        # Update particle position using Verlet integration
        temp = np.copy(self.x)
        self.x += (self.x - self.xprev) + a * dt * dt
        self.xprev[:] = temp

        # Clear force accumulator
        self.fnet[:] = 0.