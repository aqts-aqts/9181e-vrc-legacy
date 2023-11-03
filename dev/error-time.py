import matplotlib.pyplot as plt
import numpy as np

# Initialize variables
previous_error = 0
previous_position = 0  # This is the initial position of the system

# PD Controller function
def PD(kP, kD, current_position, target_position):
    global previous_error
    # Calculate the error
    error = target_position - current_position
    # Calculate the derivative of the error
    derivative = error - previous_error
    # Remember the error for the next time step
    previous_error = error
    # Return the control output and the error
    return kP * error + kD * derivative, error

# Target position
target = 100
# Time steps
dt = 0.1
# Number of steps
steps = 100
# Arrays to hold the time, position, and error values
t = np.linspace(0, dt*steps, steps)
position = np.zeros(steps)
errors = np.zeros(steps)

# Initialize the error for the first step
errors[0] = target - previous_position

# Simulation loop
for i in range(1, steps):
    # Apply the PD controller to get the control effort
    control_effort, error = PD(kP=1.5, kD=0.1, current_position=previous_position, target_position=target)
    # Record the error
    errors[i] = error
    # Update the position (this would be based on system dynamics in a real scenario)
    position[i] = position[i-1] + control_effort * dt
    # Update the previous position for the next iteration
    previous_position = position[i]

# Plot the error over time
plt.plot([], [], label='Error')
plt.yticks([20, 40, 60, 80, 100])
plt.xticks([2, 4, 6, 8, 10])
plt.title('PID Control Error Over Time')
plt.xlabel('Time')
plt.ylabel('Error')
plt.legend()
plt.grid(True)
plt.show()