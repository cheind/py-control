
import numpy as np

def run_control_loop(target, particle, ctrl, extraforces=None, tsim=10., dt=0.01):
    ts = np.arange(0., tsim, dt)

    shape = particle.x.shape
    fields = [
        ('x', np.float32, shape), 
        ('v', np.float32, shape),
        ('e', np.float32, shape),
        ('t', np.float32, 1),
        ('y', np.float32, shape)
    ]

    result = np.zeros(len(ts), dtype=fields)

    for idx, t in enumerate(ts):

        y = target(t)
        e = y - particle.x     
        u = ctrl.update(e, dt)

        # Book keeping
        r = result[idx]
        r['t'] = t
        r['x'][:] = particle.x
        r['v'][:] = particle.v
        r['y'][:] = y
        r['e'][:] = e

        # Apply control value as force
        particle.add_force(u)

        # Add any additional forces
        if extraforces is not None:
            extraforces(particle, t)

        # Update particle position and velocity
        particle.update(dt)

    return result
