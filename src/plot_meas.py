#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":
    meas = np.load("meas.npy")
    hotspots = np.load("hotspots.npy")
    odom = np.load("odom.npy")
    nn_idxs = np.load("nn_idxs.npy")
    print(nn_idxs)
    print(meas.shape, hotspots, odom.shape, nn_idxs.shape)

    FT2METER = 0.3048

    # gt = np.array([[49.833, 0, 0], [49.833, -27.583, 0], [24.833, -57.333, 0], [11.083, -41.833, 0]]) # config 1
    gt = np.array([[2.167, -18.75, 0], [30.083, -15.75, 0], [16.917, -37.33, 0], [38.5, -37.33, 0]]) # config 2
    gt *= FT2METER
    print(gt)
    # plt.scatter(-meas[:, 1], meas[:, 0], c='g')
    # plt.scatter(-hotspots[:, 1], hotspots[:, 0], c='r')
    # plt.scatter(-gt[:, 1], gt[:, 0], c='b')
    # plt.savefig('raw_meas.png')
    # exit()

    # Error polyfit
    errors = []
    gt_dists = []
    for i in range(odom.shape[0]):
        # gt_dist = np.linalg.norm(odom[i] - hotspots[nn_idxs[i]])
        # error = np.linalg.norm(hotspots[nn_idxs[i]] - meas[i])
        curr_pose = odom[i]
        reading = meas[i]
        error = np.linalg.norm(hotspots[nn_idxs[i]] - gt[nn_idxs[i]])
        gt_dists.append(gt_dist)
        errors.append(error)

    # plt.xlim(2,5)
    # plt.ylim(0,1.25)
    plt.scatter(gt_dists, errors)
    # plt.scatter(odom[:, 0], odom[:, 1])
    plt.show()