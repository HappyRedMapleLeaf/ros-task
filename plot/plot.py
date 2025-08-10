import matplotlib.pyplot as plt
import math

# sample line:
# [0m[INFO] [1754803357.749476560] [demo_robot_controller]: PLOTDATA, 9.99, 10.00, 1.55[0m
# [INFO] [1754803357.749476560] [demo_robot_controller]: PLOTDATA, 9.99, 10.00, 1.55

euclidean_errors = []
angular_errors = []
timestamps = []

with open('output.txt', 'r') as f:
    lines = f.readlines()

for i in range(len(lines)):
    line = lines[i]
    if 'PLOTDATA' in line:
        fixed_line = line.replace('\x1b[0m', '').strip()
        timestamps.append(float(fixed_line.split('[')[2].split(']')[0]))
        x = float(fixed_line.split(',')[-3])
        y = float(fixed_line.split(',')[-2])
        theta = float(fixed_line.split(',')[-1])

        euclidean_errors.append(math.sqrt((x - 10.0)**2 + (y - 10.0)**2))
        angular_errors.append(theta - 1.571)

# Convert to relative time (start from 0)
if timestamps:
    start_time = timestamps[0]
    timestamps = [t - start_time for t in timestamps]

print("Number of data points:")
print(len(timestamps), len(euclidean_errors), len(angular_errors))

euclidean_threshold = 0.05
angular_threshold = 0.1

ZOOMED = True

# plot both errors on y axis and time on x axis
plt.figure(figsize=(10, 6))
plt.plot(timestamps, euclidean_errors, 'b-', label='Euclidean Error', markersize=1)
plt.plot(timestamps, angular_errors, 'r-', label='Angular Error', markersize=1)
plt.xlabel('Time (s)')
plt.ylabel('Error')
plt.title('Robot Pose Error')

if ZOOMED:
    plt.xlim(max(timestamps) - 10, max(timestamps))
    plt.ylim(-0.2, 0.2)
    plt.axhline(y=euclidean_threshold, color='b', linestyle='--', alpha=0.7, label=f'Euclidean Threshold ({euclidean_threshold})')
    plt.axhline(y=angular_threshold, color='r', linestyle='--', alpha=0.7, label=f'Angular Threshold ({angular_threshold})')
    plt.axhline(y=-euclidean_threshold, color='b', linestyle='--', alpha=0.7)
    plt.axhline(y=-angular_threshold, color='r', linestyle='--', alpha=0.7)
else:
    plt.xlim(0, max(timestamps))
    plt.ylim(-5, 20)

plt.legend()
plt.grid(True)
plt.show()
