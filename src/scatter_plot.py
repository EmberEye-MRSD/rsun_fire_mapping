import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":
    meas = np.load("meas.npy")
    hotspots = np.load("hotspots.npy")
    print(meas.shape)
    print(hotspots.shape)

    # for i in range(hotspots.shape[0]):

        # plt.scatter(meas[i][:, 0], meas[i][:, 1], c='b')
        # plt.scatter(hotspots[:, 0], hotspots[:, 1], c='r')
    # plt.show()