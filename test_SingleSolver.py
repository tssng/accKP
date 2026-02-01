import numpy as np
import matplotlib.pyplot as plt

def read_pose_file(filename):
    """ Reads 'timestamp x y theta' format """
    data = np.loadtxt(filename)
    return data # Returns [N, 4] matrix

def get_nearest_gt(timestamp, gt_data):
    """ Finds the ground truth row with closest timestamp """
    times = gt_data[:, 0]
    idx = (np.abs(times - timestamp)).argmin()

    return gt_data[idx, :]

# 1. Load Data
# estimates = [timestamp, x, y, theta]
my_slam = read_pose_file("resultsSingle.txt")
# gt = [timestamp, x, y, theta] (Format depends on dataset, check README)
ground_truth = read_pose_file("./data/Robot1_GroundTruth.dat")

# ahift my_slam so its first point matches the GT's first point (at that time)
start_time = my_slam[0, 0]
gt_start = get_nearest_gt(start_time, ground_truth)

# calculate the transform (offset) needed
#offset_x = gt_start[1] - my_slam[0, 1]
#offset_y = gt_start[2] - my_slam[0, 2]
#offset_theta = gt_start[3] - my_slam[0, 3]

# apply offset to align trajectories
#my_slam[:, 1] += offset_x
#my_slam[:, 2] += offset_y

# calculate Error (RMSE)
errors = []
aligned_gt_x = []
aligned_gt_y = []

for row in my_slam:
    t = row[0]
    gt_row = get_nearest_gt(t, ground_truth)

    # store for plotting
    aligned_gt_x.append(gt_row[1])
    aligned_gt_y.append(gt_row[2])

    # euclidean Distance Error
    dist = np.sqrt((row[1] - gt_row[1])**2 + (row[2] - gt_row[2])**2)
    errors.append(dist)

rmse = np.sqrt(np.mean(np.array(errors)**2))
print(f"Final RMSE: {rmse:.4f} meters")

plt.figure(figsize=(10, 6))
plt.plot(ground_truth[:, 1], ground_truth[:, 2], label="Ground Truth", color='gray', linestyle='--')
plt.plot(my_slam[:, 1], my_slam[:, 2], label=f"My SLAM (RMSE={rmse:.2f}m)", color='blue')
plt.scatter(my_slam[0, 1], my_slam[0, 2], label="Start", color='green', marker='o')
plt.legend()
plt.title("Trajectory Alignment Check")
plt.axis("equal")
plt.show()