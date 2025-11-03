import matplotlib.pyplot as plt
import numpy as np

# Load data
data     = np.loadtxt(r"C:\Users\Imoge\OneDrive - UBC\Desktop\PIANOBOT\pianoBot\halldata\hall_1.txt", delimiter=",", skiprows=1)
pid_data =  np.loadtxt(r"C:\Users\Imoge\OneDrive - UBC\Desktop\PIANOBOT\pianoBot\halldata\pwm_1.txt", delimiter=",", skiprows=1)
ctrl_set = np.loadtxt(r"C:\Users\Imoge\OneDrive - UBC\Desktop\PIANOBOT\pianoBot\halldata\ctrl_1.txt", delimiter=",", skiprows=1)
# o = start, 1 = end




# print(len(data))
# print(len(pid_data))

# Hall sensor data

x = data[:, 0] /1000 # time (ms)
y = data[:, 1]  # hall voltage

# # Sort data by time
sort_indices = np.argsort(x)
x = x[sort_indices]
y = y[sort_indices]

# # PID data
pid_time = pid_data[:, 0]/1000
pid_pwm = pid_data[:, 1]


ctrl_time = ctrl_set[:,0]/1000
ctrl_pwm = ctrl_set[:,1]

# --- Plot ---
fig, ax1 = plt.subplots(figsize=(10, 5))
# t0=x[0]
# tf =4.5#x[-1]


# Left Y-axis → Hall voltage
color1 = 'tab:blue'
ax1.set_xlabel("Time (s)")
ax1.set_ylabel("Hall Voltage (V)", color=color1)
# ax1.set_xlim(t0,tf)
ax1.plot(x, y, color=color1, label="Hall Voltage", linewidth=1,)
ax1.tick_params(axis='y', labelcolor=color1)
ax1.legend(loc='upper center', bbox_to_anchor=(0.5, -0.1))

# Right Y-axis → PWM signal
ax2 = ax1.twinx()
color2 = 'tab:red'
ax2.set_ylabel("PWM Output", color=color2)
ax2.plot(pid_time, pid_pwm,  color=color2,label="PWM Output", linewidth=1, alpha=0.8)
ax2.tick_params(axis='y', labelcolor=color2)
# ax2.set_xlim(t0,tf)
ax2.legend()

## song input
ax3 = ax1.twinx()
ax3.set_ylabel("CTRL Input", color = "green")
ax3.plot(ctrl_time, ctrl_pwm, color = "green", label="CTRL Input", linewidth=1, alpha=0.8)
ax3.tick_params(axis='y', labelcolor="green")
# ax3.set_xlim(t0,tf)
ax3.legend()

# Optional: grid, legend, title
fig.suptitle("Hall Voltage vs PWM Output Over Time")
fig.tight_layout()
ax1.grid(True, which='both', linestyle='--', linewidth=0.5)
plt.show()

# print("Data plotted successfully.")
# print(len(x))

# print("avg pressed", np.mean(y[1500:]))
# print("avg released", np.mean(y[:900]))