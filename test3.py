import numpy as np

Ct = np.arange(0.0, 0.99, 0.05)

a_plus = (1 + np.sqrt(Ct))/2.
a_minus = (1 - np.sqrt(Ct))/2.

print(a_plus)
print(a_minus)