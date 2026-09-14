import numpy as np
from config import T


def F_kinematics(state, inputs):
    """
    Propagate the state according to the discrete process equation.

    State:
        state = [x, y, theta, mu1, mu2, mu3]

    Input:
        inputs = {
            'vx': body velocity along the global x direction,
            'vy': body velocity along the global y direction,
            'w' : angular velocity
        }

    Model:
        x_{k+1}      = x_k      + mu1 * vx * T
        y_{k+1}      = y_k      + mu2 * vy * T
        theta_{k+1}  = theta_k  + mu3 * w  * T
        mu1,2,3_{k+1}= mu1,2,3_k
    """

    # 1. Unpack the current state.
    x_k = state[0]
    y_k = state[1]
    theta_k = state[2]
    mu1 = state[3]
    mu2 = state[4]
    mu3 = state[5]

    # 2. Unpack the input.
    vx = inputs['vx']
    vy = inputs['vy']
    w = inputs['w']

    # 3. Compute the next state.
    x_next = x_k + mu1 * vx * T
    y_next = y_k + mu2 * vy * T
    theta_next = theta_k + mu3 * w * T

    # Treat the mu parameters as constant (random-walk model).
    mu1_next = mu1
    mu2_next = mu2
    mu3_next = mu3

    # 4. Pack the state again.
    next_state = np.array([
        x_next, y_next, theta_next,
        mu1_next, mu2_next, mu3_next
    ])

    return next_state
