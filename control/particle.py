
import numpy as np

class Particle(object):
    """A particle in n-dimensions driven by forces."""

    def __init__(self, x0=[0,0], v0=[0,0], inv_mass=1.):
        self.x = np.asarray(x0, dtype=float)
        self.v = np.asarray(v0, dtype=float)
        self.inv_mass = inv_mass
        self.fnet = np.zeros(self.x.shape)

    def add_force(self, f):
        self.fnet += f

    def update(self, dt):

        # Update particle position using explicit Euler (timestep should be sufficiently small)
        a = self.fnet * self.inv_mass
        self.x += self.v * dt
        self.v += a * dt

        # Clear force accumulator
        self.fnet[:] = 0.