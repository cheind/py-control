
import numpy as np

class PID(object):
    """Implementation of a PID controller.

    This implementation is geared towards discrete time systems,
    where PID is often called PSD (proportional-sum-difference).

    Usage is fairly straight forward. Set the coefficients of
    the three terms to values of your choice and call PID.update
    with constant timesteps.
    """
    
    def __init__(self, dim=1, kp=0.5, ki=0.0, kd=0.01):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.lastError = np.zeros(dim)
        self.sumError = np.zeros(dim)
        self.dim = dim

    def update(self, error, dt):
        """Update the PID controller.

        Computes the error as 
            err(t) = setpoint(t) - measured(t)
        
        and then computes the new control value as                 
            u(t) = kp*err(t) + kd*d/dt(err(t)) + ki*I(e)
        
        where I(e) is the integral of the error up to the current timepoint.

        Args:
            error: Error between setpoint and measured
            dt: Time step delta

        Returns:
            Returns the control value u(t) and err(t)
        """

        error = np.atleast_1d(error)
        derr = (error - self.lastError) / dt

        self.sumError += error * dt
        self.lastError[:] = error

        u = self.kp * error + self.kd * derr + self.ki * self.sumError
        
        if self.dim == 1:
            u = np.asscalar(u)

        return u