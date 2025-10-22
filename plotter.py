import matplotlib.pyplot as plt
import numpy as np



data     = np.loadtxt("C:\\Users\\Imoge\\OneDrive - UBC\\Desktop\\PIANOBOT\\pianoBot\\halldata\\hall.txt", delimiter=",", skiprows=1)#usecols=range(1, 3))
pid_data = np.loadtxt("C:\\Users\\Imoge\\OneDrive - UBC\\Desktop\\PIANOBOT\\pianoBot\\halldata\\pwm.txt", delimiter=",", skiprows=1)#usecols=range(1, 3))

# print(data.shape)
start = 0
end = len(data)

pressed = 2.43
released = 2.18

x = data[start:end, 0]#/1000  # Convert ms to s
y = data[start:end, 1]
#sort data by ascending x values
sort_indices = np.argsort(x)
x = x[sort_indices]
y = y[sort_indices]

pid_time = pid_data[:,0]
pid_pwm = pid_data[:,2]
# print(len(x), len(clean_x))
# v_ref = 5  # Reference voltage to hall
# y = adc_reading * (v_ref / 4095.0)  # Convert 12 bit esp32 ADC reading to voltage


coeffs = np.polyfit(x, y, 1)  # 1 = linear
slope, intercept = coeffs
y_fit = slope * x + intercept


# x2, y2 = data2[:, 0], data2[:, 1]
# print(x,y)
plt.plot(x, y, markersize=3, label="Hall Voltage")
plt.plot(pid_time, pid_pwm, label="PWM Output ")  # Scale PWM to 0-5V for comparison
# plt.plot(x, y_fit, 'r-', linewidth=2, label=f"Linear Fit: y= {slope:.2f}x + {intercept:.2f}")

plt.legend()
plt.xlabel("time ")
# plt.xlabel("position ")

plt.ylabel(" Voltage (V)")
# plt.title("Hall Data,  magnet, VC ONN")
plt.show()

# plot_data(x2, y2, title="Hall Data VC OFF", xlabel="time", ylabel="Hall")
# plot_data(data2[:,0], data2[:,1], title="Hall Data Plot 2", xlabel="time", ylabel="voltage")
print("Data plotted successfully.")
# plt.savefig("C:\\Users\\Imoge\\OneDrive - UBC\\Desktop\\PIANOBOT\\pianoBot\\halldata\\hall_data_ploton.png")
