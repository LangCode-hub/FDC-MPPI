import numpy as np

# ===== Basic parameters =====
T = 0.01                    # Sampling period (s)

WINDOW_SIZE = 8             # Window length
STATE_DIM = 6               # State dimension: [x, y, theta, mu1, mu2, mu3] [cite: 1]

# ===== Measurement weight =====
W_POS = 1.0                 # Measurement-residual weight for x, y, and theta

# ===== Per-component step sizes (6D) =====
# State order: x, y, theta, mu1, mu2, mu3
# The first three are motion states; the last three are parameters.
ALPHA = np.array([
    0.05, 0.05, 0.05,          # Update step sizes for x, y, and theta
    50, 50, 10        # Update step sizes for mu1, mu2, and mu3
], dtype=float)
