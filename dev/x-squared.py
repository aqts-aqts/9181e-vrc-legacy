import matplotlib.pyplot as plt
import numpy as np

# Create an array of x values from -10 to 10
x = np.linspace(-10, 10, 400)

# Calculate the corresponding y values for each x value
y = x**2

# Plotting the curve
plt.figure(figsize=(8,6))
plt.plot(x, y, label="y = x^2")
plt.title("Graph of y = x^2")
plt.xlabel("x")
plt.ylabel("y")
plt.grid(True)
plt.legend()
plt.show()