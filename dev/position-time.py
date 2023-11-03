import matplotlib.pyplot as plt
import numpy as np

# Initialize variables
previous_error = 0
previous_position = 0

# PD Controller function
def PD(kP, kD, current_position, target_position):
    global previous_error
    # Calculate the error
    error = target_position - current_position
    # Calculate the derivative of the error
    derivative = error - previous_error
    # Remember the error for the next time step
    previous_error = error
    # Return the control output, which is the sum of the proportional and derivative terms
    return kP * error + kD * derivative

# Target position
target = 100
# Time steps
dt = 0.1
# Number of steps
steps = 100
# Arrays to hold the time and position values
t = np.linspace(0, dt*steps, steps)
position = np.zeros(steps)

# Simulation loop
for i in range(1, steps):
    # Apply the PD controller to get the control effort
    control_effort = PD(kP=1.5, kD=0.1, current_position=previous_position, target_position=target)
    # For a realistic simulation, apply the control effort to change the system's state
    # Here, we just simulate this by adding the control effort to the current position
    # In a real system, you might have a more complex relationship here
    position[i] = position[i-1] + control_effort * dt
    # Update the previous position
    previous_position = position[i]

# Plot the position over time
plt.plot(t, position, label='Position')
plt.plot(t, np.full(steps, target), 'r--', label='Target')
plt.title('Smoother speed descent with optimal kD')
plt.xlabel('Time')
plt.ylabel('Position')
plt.legend()
plt.grid(True)
plt.show()
