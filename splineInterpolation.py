import numpy as np
import matplotlib.pyplot as plt

# Given points
x = np.array([30, 40, 50, 60, 70, 80, 90, 100])
y = np.array([6.6, 8.5, 8.5, 12.2, 14, 16, 18, 20])

# Number of intervals
n = len(x) - 1

# Calculate the step sizes h_i
h = np.diff(x)

# Calculate the system matrix A and vector b
A = np.zeros((n+1, n+1))
b = np.zeros(n+1)

# Boundary conditions for natural spline (second derivatives at endpoints are zero)
A[0, 0] = 1
A[n, n] = 1

# Set up the system of equations for the interior points
for i in range(1, n):
    A[i, i-1] = h[i-1]
    A[i, i] = 2 * (h[i-1] + h[i])
    A[i, i+1] = h[i]
    b[i] = 3 * ((y[i+1] - y[i]) / h[i] - (y[i] - y[i-1]) / h[i-1])

# Solve for the second derivatives
M = np.linalg.solve(A, b)

# Calculate the coefficients for each interval
a = y[:-1]
b_coef = (y[1:] - y[:-1]) / h - h * (2 * M[:-1] + M[1:]) / 3
c = M[:-1]
d = (M[1:] - M[:-1]) / (3 * h)

# Print the coefficients
for i in range(n):
    print(f"Interval [{x[i]}, {x[i+1]}]: a={a[i]}, b={b_coef[i]}, c={c[i]}, d={d[i]}")

# Function to evaluate the spline
def evaluateSpline(x_val):
    for i in range(n):
        if x_val >= x[i] and x_val <= x[i+1]:
            dx = x_val - x[i]
            return a[i] + b_coef[i] * dx + c[i] * dx * dx + d[i] * dx * dx * dx
    return np.nan  # x is out of the interpolation range

# Generate points for plotting the spline
x_vals = np.linspace(x[0], x[-1], 200)
y_vals = [evaluateSpline(xi) for xi in x_vals]

# Plot the given points and the fitted spline
plt.figure()
plt.plot(x, y, 'o', label='Given Points')
plt.plot(x_vals, y_vals, '-', label='Cubic Spline')
plt.xlabel('x')
plt.ylabel('y')
plt.title('Cubic Spline Interpolation')
plt.legend()
plt.grid(True)
plt.show()

