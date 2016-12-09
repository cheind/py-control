
import numpy as np
import matplotlib.pyplot as plt
import control as ctrl


class MoveParticleProcess(ctrl.Process):

    def __init__(self, particle=ctrl.Particle(), pid=ctrl.PID()):
        super(MoveParticleProcess, self).__init__()
        self.particle = particle
        self.pid = pid

    def target(self, t):
        """Return setpoint position for particle to reach.
        Simple step function at t == 1. and t==15.
        """
        if t < 1. or t >= 15.:
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

def run():

    # Various parameters to run simulation at
    pid_params = [
        dict(kp=1.5, ki=0., kd=0.),
        dict(kp=1.5, ki=0., kd=0.5),
        dict(kp=1.5, ki=0., kd=1.5),
    ]

    # Run simulation for each set of PID params
    handles = []
    for idx, c in enumerate(pid_params):
        process = MoveParticleProcess(particle=ctrl.Particle(x0=[0], v0=[0], inv_mass=1.), pid=ctrl.PID(**c))
        result = process.loop(tsim=50, dt=0.1)

        e = np.sum(np.square(result['e']))
        print(e)

        if idx == 0:
            fh, = plt.step(result['t'], result['y'], label='target')    
            handles.append(fh)    

        xh, = plt.plot(result['t'], result['x'], label='pid kp{} kd{} ki{}'.format(c['kp'], c['kd'], c['ki']))
        handles.append(xh)

    # Plot the target
    
    plt.title('Particle trajectory')
    plt.legend(handles=handles, loc=1)
    plt.xlabel('Time $sec$')
    plt.ylabel('Position $m$')
    plt.show()
    

if __name__ == "__main__":
    run() 

