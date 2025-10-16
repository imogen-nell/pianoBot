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


data = np.loadtxt("C:\\Users\\Imoge\\OneDrive - UBC\\Desktop\\PIANOBOT\\pianoBot\\halldata\\hall_data.txt", delimiter=",", skiprows=1,usecols=range(1, 3))
# data2 = np.loadtxt("C:\\Users\\Imoge\\OneDrive - UBC\\Desktop\\PIANOBOT\\pianoBot\\halldata\\hall_data2.txt", delimiter=",", skiprows=1)

print(data.shape)




x = data[:, 0]
y = data[:, 1]

# print(x,y)
plot_data(x, y)
# plot_data(data2[:,0], data2[:,1], title="Hall Data Plot 2", xlabel="time", ylabel="voltage")
print("Data plotted successfully.")
plt.savefig("C:\\Users\\Imoge\\OneDrive - UBC\\Desktop\\PIANOBOT\\pianoBot\\halldata\\hall_data_plot.png")