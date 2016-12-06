
"""PID controller to estimate acceleration for target position.

A PID controller is used to estimate necessary acceleration to
reach a target position. The target position can either be stationary
or dynamic.

The motion equations are integrated by explicit Euler steps using
the PID controller value as acceleration for every timestamp.
"""

import numpy as np
import matplotlib.pyplot as plt

from control.pid import PID

def f(t):
    return 0. if t < 1. else 1.1
    # Also try np.sin(t) and other

def run_simulation(fs, x0=0, v0=0, dt=0.05, kp=1.5, ki=0, kd=0.):

    pid = PID(kp=kp, ki=ki, kd=kd)

    xs = np.zeros(len(fs))

    # Start solution
    xs[0] = x0  # Initial position
    v = v0      # Initial velocity

    sum_errors = 0.

    for i in range(len(fs)-1):
        # Update position, velocity using explicit euler
        xs[i+1] = xs[i] + v * dt
        e = fs[i+1] - xs[i+1]     
        u = pid.update(e, dt)
        v = v + u * dt

        sum_errors += e

    return sum_errors / len(fs), xs

def run():
    dt = 0.05
    ts = np.arange(0, 50, dt)
    fs = [f(t) for t in ts]

    # Various parameters to run simulation at
    combinations = [
        dict(kp=1.5, ki=0., kd=0.),
        dict(kp=1.5, ki=0., kd=0.5),
        dict(kp=1.5, ki=0., kd=1.5)
    ]
    
    # Plot the target
    handles = []
    fh, = plt.step(ts, fs, label='x*')    
    handles.append(fh)    
    
    # Run simulation for each combination
    for c in combinations:
        err, xs = run_simulation(fs, dt=dt, **c)
        xh, = plt.plot(ts, xs, label='x kp{} kd{} ki{}'.format(c['kp'], c['kd'], c['ki']))
        handles.append(xh)

    plt.title('PID Controller')
    plt.legend(handles=handles)
    plt.ylim([-5,5])
    plt.show()

        
if __name__ == "__main__":
    run() 
