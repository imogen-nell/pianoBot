import matplotlib.pyplot as plt
import numpy as np

def plot_data(x, y, title="Hall Data Plot", xlabel="time", ylabel="voltage"):
    plt.figure()
    plt.plot(x, y)
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.grid()
    plt.show()


data = np.loadtxt("C:\\Users\\Imoge\\OneDrive - UBC\\Desktop\\PIANOBOT\\pianoBot\\halldata\\hall_data_10.txt", delimiter=",", skiprows=1)



x = data[:, 0]
y = data[:, 1]

# print(x,y)
plot_data(x, y)
print("Data plotted successfully.")
plt.savefig("C:\\Users\\Imoge\\OneDrive - UBC\\Desktop\\PIANOBOT\\pianoBot\\halldata\\hall_data_plot.png")