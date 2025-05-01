import matplotlib.pyplot as plt

# Simulation Parameters
dt = 0.1               # control loop interval (s)
time_end = 10.0        # total simulation time (s)
steps = int(time_end / dt)

# Controller Gains
KP = 400
KI = 17
KF = 80               # Feedforward gain
MAX_SPEED = 0.5       # Max expected speed in m/s
MIN_FEEDFORWARD = 60
MIN_PWM = 30
MAX_PWM = 250

# Desired speed (you can change this)
desired_speed = 0.3  # m/s

# Initial states
actual_speed = 0.0
integral = 0.0

# Simulated motor response function
def motor_response(pwm, current_speed):
    static_friction_pwm = 200
    if abs(pwm) < static_friction_pwm:
        return current_speed * 0.9  # slow decay if stuck
    else:
        target_speed = (pwm - 80) * 0.005  # adjust gain for your setup
        return current_speed + 0.2 * (target_speed - current_speed)

# Data logging
history = []

# Simulation loop
for step in range(steps):
    error = desired_speed - actual_speed
    integral += error * dt

    # Feedforward
    ff = max(KF * (abs(desired_speed) / MAX_SPEED), MIN_FEEDFORWARD)
    ff *= 1 if desired_speed > 0 else -1

    # PI controller output
    pwm = KP * error + KI * integral + ff

    # Clamp PWM to bounds
    pwm = max(min(pwm, MAX_PWM), -MAX_PWM)
    if 0 < pwm < MIN_PWM:
        pwm = MIN_PWM
    elif -MIN_PWM < pwm < 0:
        pwm = -MIN_PWM

    # Simulate motor dynamics
    actual_speed = motor_response(pwm, actual_speed)

    # Save data
    history.append((step * dt, desired_speed, actual_speed, pwm, error))

# Unpack data
times, desired_vals, actual_vals, pwm_vals, errors = zip(*history)

# Plotting
plt.figure()
plt.plot(times, desired_vals, label="Desired Speed")
plt.plot(times, actual_vals, label="Actual Speed")
plt.xlabel("Time (s)")
plt.ylabel("Speed (m/s)")
plt.title("Speed Response")
plt.legend()
plt.grid(True)

plt.figure()
plt.plot(times, pwm_vals, label="PWM Signal")
plt.xlabel("Time (s)")
plt.ylabel("PWM")
plt.title("PWM Output Over Time")
plt.legend()
plt.grid(True)

plt.figure()
plt.plot(times, errors, label="Speed Error")
plt.xlabel("Time (s)")
plt.ylabel("Error (m/s)")
plt.title("Error Over Time")
plt.legend()
plt.grid(True)

plt.show()
