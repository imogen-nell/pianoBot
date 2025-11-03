import matplotlib.pyplot as plt
import numpy as np


PRESSED = 1.81
RELEASED = 1.08
# Load data
data     = np.loadtxt(r"C:\Users\Imoge\OneDrive - UBC\Desktop\PIANOBOT\pianoBot\halldata\hall_11.txt", delimiter=",", skiprows=1)
ideal = np.loadtxt(r"C:\Users\Imoge\OneDrive - UBC\Desktop\PIANOBOT\pianoBot\halldata\ctrl_11.txt", delimiter=",", skiprows=1)
pwm_pid = np.loadtxt(r"C:\Users\Imoge\OneDrive - UBC\Desktop\PIANOBOT\pianoBot\halldata\pwm_11.txt", delimiter=",", skiprows=1)

#hall 4: pid controlled : 0.5 0 0 
#hall 5: pid controlled : 0.5 0.0 0.1
#hall 6: pid controlled : 0.5 0.0 0.05
#hall 7: pid controlled : 0.5 0.02 0.0
#hall 8                   0.8 0.05 0.0
#hall 9: pid controlled : 0.5 0.05 0.0
#hall 10: pid controlled : 0.5 0.1 0.0
#hall 11: pid controlled : 0.5 0.1 0.0
#hall 12: pid controlled : 0.5 0.1 0.005
time = data[:, 0] /1000 # time (ms)
actual_V = data[:, 1]  # hall voltage

ideal_time = ideal[:,0]/1000
ideal_V = ideal[:,1]

pid_time = pwm_pid[:,0]/1000
pid_pwm = pwm_pid[:,1]


#convert pid set PWM to voltage
# m = (PRESSED-RELEASED)/255.0
# b = RELEASED
# ideal_V = [pwm * m + b for pwm in ideal_PWM]


#plot
fig, ax1 = plt.subplots(figsize=(10, 5))

color1 = 'tab:blue'
ax1.set_xlabel("Time (s)")
ax1.set_ylabel("Hall Voltage (V)", color=color1)
# ax1.set_xlim(2,5)
ax1.plot(time, actual_V, color=color1, label="Hall Voltage", linewidth=1,)
ax1.plot(ideal_time, ideal_V, color="green", label="Set Point Voltage", linewidth=1,)
ax1.tick_params(axis='y', labelcolor=color1)
ax1.legend()

ax2 = ax1.twinx()
color2 = 'tab:red'
ax2.set_ylabel("PWM Output", color=color2)
ax2.plot(pid_time, pid_pwm,  color=color2,label="PWM Output", linewidth=1, alpha=0.8)
ax2.tick_params(axis='y', labelcolor=color2)
# ax2.set_xlim(t0,tf)
ax2.legend()

plt.show()