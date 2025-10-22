import matplotlib.pyplot as plt
import numpy as np

# Load data
data     = np.loadtxt(r"C:\Users\Imoge\OneDrive - UBC\Desktop\PIANOBOT\pianoBot\halldata\hall.txt", delimiter=",", skiprows=1)
pid_data = np.loadtxt(r"C:\Users\Imoge\OneDrive - UBC\Desktop\PIANOBOT\pianoBot\halldata\pwm.txt", delimiter=",", skiprows=1)

start = 0
end = len(data)

pressed = 2.43
released = 2.18

# Hall sensor data
x = data[start:end, 0]  # time (ms)
y = data[start:end, 1]  # hall voltage

# Sort data by time
sort_indices = np.argsort(x)
x = x[sort_indices]
y = y[sort_indices]

# PID data
pid_time = pid_data[:, 0]
pid_pwm = pid_data[:, 2]

# --- Plot ---
fig, ax1 = plt.subplots(figsize=(10, 5))

# Left Y-axis → Hall voltage
color1 = 'tab:blue'
ax1.set_xlabel("Time (ms)")
ax1.set_ylabel("Hall Voltage (V)", color=color1)
ax1.plot(x, y, color=color1, label="Hall Voltage", linewidth=1)
ax1.tick_params(axis='y', labelcolor=color1)

# Right Y-axis → PWM signal
ax2 = ax1.twinx()
color2 = 'tab:red'
ax2.set_ylabel("PWM Output", color=color2)
ax2.plot(pid_time, pid_pwm, color=color2, label="PWM Output", linewidth=1, alpha=0.8)
ax2.tick_params(axis='y', labelcolor=color2)

# Optional: grid, legend, title
fig.suptitle("Hall Voltage vs PWM Output Over Time")
fig.tight_layout()
ax1.grid(True, which='both', linestyle='--', linewidth=0.5)
plt.show()

print("Data plotted successfully.")
