import matplotlib.pyplot as plt
import numpy as np

# Load data
# data     = np.loadtxt(r"C:\Users\Imoge\OneDrive - UBC\Desktop\PIANOBOT\pianoBot\halldata\hall_6.txt", delimiter=",", skiprows=1)
pid_data = np.loadtxt(r"C:\Users\Imoge\OneDrive - UBC\Desktop\PIANOBOT\pianoBot\halldata\pwm_6.txt", delimiter=",", skiprows=1)
# song_start_end_data = np.loadtxt(r"C:\Users\Imoge\OneDrive - UBC\Desktop\PIANOBOT\pianoBot\halldata\ctrl_6.txt", delimiter=",", skiprows=1)
# o = start, 1 = end

# start = 0
# end = int(len(data))


# print(len(data))
# print(len(pid_data))

# Hall sensor data
# x = data[start:end, 0] /1000 # time (ms)
# y = data[start:end, 1]  # hall voltage
# x = data[:, 0] /1000 # time (ms)
# y = data[:, 1]  # hall voltage

# # Sort data by time
# sort_indices = np.argsort(x)
# x = x[sort_indices]
# y = y[sort_indices]

# PID data
pid_time = pid_data[:, 0]/1000
pid_pwm = pid_data[:, 1]


# ctrl_time = song_start_end_data[:,0]/1000
# ctrl_pwm = song_start_end_data[:,1]

# --- Plot ---
fig, ax1 = plt.subplots(figsize=(10, 5))
t0=pid_time[0]
tf =pid_time[-1]

# Left Y-axis → Hall voltage
color1 = 'tab:blue'
ax1.set_xlabel("Time (s)")
ax1.set_ylabel("Hall Voltage (V)", color=color1)
ax1.set_xlim(t0,tf)
# ax1.plot(x, y, color=color1, label="Hall Voltage", linewidth=1)
# ax1.scatter(ctrl_time,ctrl_pwm, label = "song playing =")
ax1.tick_params(axis='y', labelcolor=color1)

# Right Y-axis → PWM signal
ax2 = ax1.twinx()
color2 = 'tab:red'
ax2.set_ylabel("PWM Output", color=color2)
# ax2.plot(ctrl_time, ctrl_pwm, color=color2, label="CTRL Output", linewidth=1, alpha=0.8)
ax2.plot(pid_time, pid_pwm,  color=color2,label="PWM Output", linewidth=1, alpha=0.8)
ax2.tick_params(axis='y', labelcolor=color2)
ax2.set_xlim(t0,tf)


ax2.legend()
# Optional: grid, legend, title
fig.suptitle("Hall Voltage vs PWM Output Over Time")
fig.tight_layout()
ax1.grid(True, which='both', linestyle='--', linewidth=0.5)
plt.show()

print("Data plotted successfully.")
