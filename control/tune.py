import numpy as np
import sys

def tune_twiddle(params, algorithm, dparams=None, eps=0.001):
    """Tune parameters using Twiddle algorithm.
    
    Twiddle, also known as coordinate ascent, is a line search algorithm
    to find an optimized set of parameters. Basic idea is to treat each
    parameter individually and search along the line with increasing steps
    as long as the error improves. If the error does not improve, one tries
    the opposite direction. If this fails as well, one decreases the step size.

    Args:
        params: Dicitionary of parameters with initial values
        algorithm: Function that takes a dictionary of parameters and returns an error / fitness value
        eps: Stop iteration if error does not improve by at least this value
        dparams: Optional dictionary of step sizes in each parameter direction. If not specified, set
                 to one for each parameter.

    By no means its guaranteed to find an optimal solution.
    """

    if dparams is None:
        dparams = dict((key, 1.) for (key, _) in params.items())

    errprev = sys.float_info.max
    err = sys.float_info.max / 2

    while (errprev - err) > eps:
        errprev = err

        for k in params.keys():
            params[k] += dparams[k]
            e = algorithm(params)

            if e < err:
                err = e
                dparams[k] *= 1.1
            else:
                params[k] -= 2 * dparams[k]
                e = algorithm(params)

                if e < err:
                    err = e
                    dparams[k] *= 1.1
                else:
                    params[k] += dparams[k]
                    dparams[k] *= 0.95
    
    return params