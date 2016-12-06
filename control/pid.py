
import numpy as np

class PID(object):
    """Implementation of a PID controller.

    This implementation is geared towards discrete time systems,
    where PID is often called PSD (proportional-sum-difference).

    Usage is fairly straight forward. Set the coefficients of
    the three terms to values of your choice and call PID.update
    with constant timesteps. PID.update returns n-dimenional control
    value.
    """
    
    def __init__(self, dim=1, kp=0.5, ki=0.0, kd=0.01):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.lastError = np.zeros(dim)
        self.sumError = np.zeros(dim)
        self.is_scalar = dim == 1

    def update(self, error, dt):
        """Update the PID controller.

            u(t) = kp*err(t) + kd*d/dt(err(t)) + ki*I(e)
        
        where I(e) is the integral of the error up to the current timepoint.
        """
        
        error = np.atleast_1d(error)        
        self.sumError += error * dt

        u = self.kp * error + self.kd * (error - self.lastError) / dt + self.ki * self.sumError

        self.lastError[:] = error

        return np.asscalar(u) if self.is_scalar else u