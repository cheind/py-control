
import numpy as np

import matplotlib.pyplot as plt

import control as ctrl

def target(t):
    # A sharp step function at t == 2
    return np.asarray([0.]) if t < 1. else np.array([1.])

def negforce(particle, t):
    particle.add_force([-1.2])

def noforce(particle, t):
    pass

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
        particle = ctrl.Particle(x0=[0], v0=[0], inv_mass=1.)
        pid = ctrl.PID(**c)
        result = ctrl.run_control_loop(target, particle, pid, extraforces=noforce, tsim=20., dt=0.001)

        if idx == 0:
            fh, = plt.step(result['t'], result['y'], label='y*')    
            handles.append(fh)    

        xh, = plt.plot(result['t'], result['x'], label='x kp{} kd{} ki{}'.format(c['kp'], c['kd'], c['ki']))
        handles.append(xh)

    # Plot the target
    
    plt.title('Position control')
    plt.legend(handles=handles, loc=1)
    plt.xlabel('Time $sec$')
    plt.ylabel('Position $m$')
    #plt.ylim([-2,8])
    plt.show()
    

if __name__ == "__main__":
    run() 

