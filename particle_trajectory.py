
import numpy as np
import matplotlib.pyplot as plt
import control as ctrl


class MoveParticleProcess(ctrl.Process):
    """Models a dynamic system in which a particle driven by forces follows a desired trajectory. """

    def __init__(self, particle=ctrl.Particle(), pid=ctrl.PID()):
        super(MoveParticleProcess, self).__init__()
        self.particle = particle
        self.pid = pid

    def target(self, t):
        """Return setpoint position for particle to reach.
        Simple step function at t == 1. and t==15.
        """
        if t < 5. or t >= 15.:
            return np.asarray([0.])
        else:
            return np.array([1.])
    
    def sense(self, t):
        """Sense particle position."""
        return self.particle.x
    
    def correct(self, error, dt):
        """Compute correction based on error."""
        return self.pid.update(error, dt)

    def actuate(self, u, dt):
        """Update particle position. 
        Takes the correction value `u` and interprets it as force acting on the particle, 
        then upates the motion equations by `dt`.
        """
        self.particle.add_force(u)
        self.particle.update(dt)


def runner(pid_params):
    process = MoveParticleProcess(particle=ctrl.Particle(x0=[0], v0=[0], inv_mass=1.), pid=ctrl.PID(**pid_params))
    result = process.loop(tsim=50, dt=0.2)
    e = np.sum(np.square(result['e']))
    return e

def run():

    # Various PID controller parameters to run simulation with 
    pid_params = [
        dict(kp=0.1, ki=0., kd=0.),
        dict(kp=1.5, ki=0., kd=0.5),
    ]

    # Additionally tune PID parameters
    params = ctrl.tune_twiddle(params=dict(kp=0., ki=0., kd=0.), costfunction=runner, eps=0.001)
    pid_params.append(params)

    # Run simulation for each set of PID params
    handles = []
    for idx, c in enumerate(pid_params):
        process = MoveParticleProcess(particle=ctrl.Particle(x0=[0], v0=[0], inv_mass=1.), pid=ctrl.PID(**c))
        result = process.loop(tsim=100, dt=0.1)

        if idx == 0:
            fh, = plt.step(result['t'], result['y'], label='target')    
            handles.append(fh)    

        xh, = plt.plot(result['t'], result['x'], label='pid kp {:.2f} kd {:.2f} ki {:.2f}'.format(c['kp'], c['kd'], c['ki']))
        handles.append(xh)
    
    plt.title('Particle trajectory')
    plt.legend(handles=handles, loc=1)
    plt.xlabel('Time $sec$')
    plt.ylabel('Position $m$')
    plt.show()
    

if __name__ == "__main__":
    run() 

