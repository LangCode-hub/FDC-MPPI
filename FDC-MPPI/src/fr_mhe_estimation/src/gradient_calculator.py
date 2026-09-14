import numpy as np
from config import T, W_POS, STATE_DIM


class GradientCalculator:
    """
    State: x = [x, y, theta, mu1, mu2, mu3]
    Measurement: z = [x, y, theta]
    """

    def __init__(self, window_size, state_dim=STATE_DIM):
        self.window_size = window_size
        self.state_dim = state_dim

        # Measurement matrix H (3x6), corresponding to x, y, and theta.
        self.H = np.zeros((3, state_dim))
        self.H[0, 0] = 1.0  # x
        self.H[1, 1] = 1.0  # y
        self.H[2, 2] = 1.0  # theta

    def jacobian_f(self, xk, uk):
        """
        Compute the Jacobian of process equation f with respect to state x (6x6).

        State:
            x = [x, y, theta, mu1, mu2, mu3]
        Input:
            u = {vx, vy, w}
        Model:
            x_{k+1}      = x_k      + mu1 * vx * T
            y_{k+1}      = y_k      + mu2 * vy * T
            theta_{k+1}  = theta_k  + mu3 * w  * T
        """

        # Extract state variables used to compute partial derivatives.
        mu1 = xk[3]
        mu2 = xk[4]
        # mu3 = xk[5]

        vx = uk['vx']
        vy = uk['vy']
        w = uk['w']

        # Initialize as an identity matrix (ones on the diagonal).
        J = np.eye(self.state_dim)

        # --- Row 0: partial derivatives of x_{k+1} with respect to the state ---
        # x_next = x + mu1 * vx * T
        # There is no theta dependency, so J[0, 2] remains zero.
        # d(x_next)/d(mu1)
        J[0, 3] = vx * T

        # --- Row 1: partial derivatives of y_{k+1} with respect to the state ---
        # y_next = y + mu2 * vy * T
        # This is likewise independent of theta.
        # d(y_next)/d(mu2)
        J[1, 4] = vy * T

        # --- Row 2: partial derivatives of theta_{k+1} with respect to the state ---
        # theta_next = theta + mu3 * w * T
        # d(theta_next)/d(mu3)
        J[2, 5] = w * T

        # The other self-derivatives of x, y, and theta are already in the identity matrix.
        return J

    def compute_gradient(self, states, inputs, measurements):
        """
        Compute the gradient using the Jacobian matrix.
        For the cost function:
            J = sum_k W_POS * || H x_k - z_k ||^2
        compute the gradient with respect to the state x_0 at the start of the window.
        """
        # Composite Jacobian DF_k = d x_k / d x_0.
        DF = np.eye(self.state_dim)
        grad = np.zeros(self.state_dim)

        # The measurements dictionary must contain 'pose' for [x, y, theta].
        meas_data = measurements['pose']

        for k in range(self.window_size):
            # 1) Current residual r_k = x_k(0:3) - z_k.
            r = states[k][0:3] - meas_data[k]

            # 2) Accumulate the gradient using the current DF_k.
            #   grad += 2 * DF_k^T * H^T * (W_POS * r_k)
            grad += DF.T @ self.H.T @ (W_POS * r)

            # 3) Recursively compute DF_{k+1} = J_f(x_k, u_k) * DF_k.
            Jf = self.jacobian_f(states[k], inputs[k])
            DF = Jf @ DF

        return 2.0 * grad
