import numpy as np

## import the .npy file and print the poses

# poses = np.load('magic_gangu.npy')
poses = np.load('raw_data_config2.npy')
# poses = np.load("nardoconfig2.npy")
# config2_poses = np.load('config2.npy')
# config2_poses = np.load('magic_gangu.npy')

poses_fitted = np.load('fitted_plot.npy')
# disp = np.load('disp_config2.npy')

# print(np.min(disp), np.max(disp))
# print(np.median(disp))

# print(len(poses[0]))
# print(poses)

## plot the scatter plot of the poses
import matplotlib.pyplot as plt


## a function to calculate the closest coordinate in gt_arr to poses[0]
def closest_point(poses, gt_arr):
    min_dist = 1000000
    min_index = 0
    for i in range(len(gt_arr)):
        # print(poses, gt_arr[i])
        dist = np.linalg.norm(poses - gt_arr[i])
        if dist < min_dist:
            min_dist = dist
            min_index = i
    return min_index



# plt.scatter(-gangu[0][:,1], gangu[0][:,0])
# filter_poses = gangu[0][gangu[0][:,2] < 0]
# filter_poses = filter_poses[filter_poses[:,2] > -0.52]
# get indexes of the poses where Z > 0

####################### Plotting raw and fitted poses ################
######################################################################
idxs = np.where(poses[0][:,2] < 0)
filter_poses = poses[0][idxs]
filter_poses_imu = poses[1][idxs]
# filter_poses = poses[0][poses[0][:,2] < 0]

plt.scatter(-filter_poses[:,1], filter_poses[:,0])

filter_poses_fitted = poses_fitted[0][poses_fitted[0][:,2] < 0]
plt.scatter(-filter_poses_fitted[:,1], filter_poses_fitted[:,0])
######################################################################
######################################################################

# gt_arr = np.array([[14.44,0], [14.44, -7.98], [7.43, -16.51], [3.96, -11.2]])

# gt_arr = np.array([[49.833,0, 0.32], [49.833, -27.583, 0.32], [24.833, -57.333, 0.32], [11.083, -41.833, 0.32]])

gt_arr = np.array([ [30.083, -15.75], [38.5, -37.33], [16.917, -37.33], [2.167, -18.75]])
gt_arr *= 0.3048
# gt_arr_config2 = gt_arr 
# gt_arr_ft = np.array()

plt.scatter(-gt_arr[:,1], gt_arr[:,0], c='r')
plt.xlabel('x')
plt.ylabel('y')
plt.title('Original ')
plt.show()

############################################################
############################################################
# gt_arr = gt_arr[0]

pose_imu_3d = filter_poses_imu #pose_imu_3d
pose_3d = filter_poses #pose_3d

print("filtered", len(pose_imu_3d), len(pose_3d))

error_y = []
error_pose3d_x = []
error_pose3d_y = []
error_pose3d_z = []

# print(pose_3d[0])
# print(pose_imu_3d)
h1, h2, h3, h4 = 0, 0, 0, 0

print("Ground Truths", gt_arr)
for i in range(len(pose_3d)):
    # print(pose_imu_3d[i], pose_3d[i], gt_arr)
    point = np.array(pose_3d[i])
    # print(point)
    closest_hotspot = closest_point(point, gt_arr)
    # print(closest_hotspot)

    if (closest_hotspot == 0):
        h1 += 1
    elif (closest_hotspot == 1):
        h2 += 1
    elif (closest_hotspot == 2):
        h3 += 1
    elif (closest_hotspot == 3):
        h4 += 1
        
    # print(pose_3d[i], gt_arr[closest_hotspot])
    error_temp = np.array([pose_3d[i][0], pose_3d[i][1]]) - np.array([gt_arr[closest_hotspot][0], gt_arr[closest_hotspot][1]])
    error_y.append(np.linalg.norm(error_temp))
    error_pose3d_x.append(pose_3d[i][0] - gt_arr[closest_hotspot][0])
    error_pose3d_y.append(pose_3d[i][1] - gt_arr[closest_hotspot][1])
    error_pose3d_z.append(pose_3d[i][2] - gt_arr[closest_hotspot][2])


# plt.plot(error_y)
# plt.xlabel('time')
# plt.ylabel('error')
# plt.title('Estimate - Ground truth of hotspot')
# plt.show()









#### Drone from Ground truth of hotspot
dist_drone_est = np.linalg.norm(pose_imu_3d, axis=1)

error_drone_gt = dist_drone_est - error_y
# plt.plot(error_drone_gt, c='r')
# # plt.plot(dist_drone_est, c='g')
# plt.xlabel('time')
# plt.ylabel('error')
# plt.title('Drone from Ground truth of hotspot')
# plt.show()









print("For hotspot 1: ", h1)
print("For hotspot 2: ", h2)
print("For hotspot 3: ", h3)
print("For hotspot 4: ", h4)
error_x = np.linalg.norm(pose_imu_3d, axis=1)

# print(len(error_x), len(error_y))
# print(error_x[80], error_y[80])

# plt.scatter(error_x , error_y)\

# imu wali
h1_time_step = 611

error_h1 = error_x[:h1_time_step]
error_h2 = error_x[228:385]
error_h3 = error_x[385:605]
error_h4 = error_x[605:]
plt.scatter(error_h1, error_y[:h1_time_step], c='r')
# plt.scatter(error_h2, error_y[228:385], c='g')
# plt.scatter(error_h3, error_y[385:605], c='b')
# plt.scatter(error_h4, error_y[605:], c='y')

plt.xlabel('distance of drone from estimates fire')
plt.ylabel('error of estimate from groundtruth')
plt.title('Magic')
# plt.show()

## remove outliers above 2SDs from the mean for error_y

# mean_error = np.mean(error_y[:228])
# std_error = np.std(error_y[:228])
# print(mean_error, std_error)
# # error_y = np.array(error_y)
# error_y = error_y[error_y[:228] < mean_error + 2*std_error]




# magic_func = np.polyfit(error_h1 , error_y[:228], deg=2)

error_h1_imu_x = pose_imu_3d[:h1_time_step, 0]
error_h1_imu_y = pose_imu_3d[:h1_time_step, 1]
error_h1_imu_z = pose_imu_3d[:h1_time_step, 2]

# print(len(error_h1_imu_x), len(error_pose3d_x))

# fixmy_x = np.polyfit(error_h1_imu_x, error_pose3d_x[:177], deg=2)
# fixmy_y = np.polyfit(error_h1_imu_y, error_pose3d_y[:177], deg=2)
# fixmy_z = np.polyfit(error_h1_imu_z, error_pose3d_z[:177], deg=2)

fixmy_x = np.polyfit(error_h1, error_pose3d_x[:h1_time_step], deg=1)
fixmy_y = np.polyfit(error_h1, error_pose3d_y[:h1_time_step], deg=1)
fixmy_z = np.polyfit(error_h1, error_pose3d_z[:h1_time_step], deg=1)



# print(magic_func)
# print (np.min(error_x))

## plot the function created above onto the scatter plot
x = np.linspace(np.min(error_x), 7, 1000)
# y = np.polyval(magic_func, x)
# plt.plot(x, y, c='r')
# plt.show()




# print(fixmy_x, fixmy_y, fixmy_z)
# np.save('fixmy_errors.npy', [fixmy_x, fixmy_y, fixmy_z])











