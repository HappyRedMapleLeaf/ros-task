## ROS2 Task - Evan Li
### Summary of Changes
- Created `controller` node in limo_control package that publishes to cmd_vel values that move the robot from its estimated current position (from odom - published by the simulation) to a destination/target position (from target - published by, in this case, the `ros2 topic pub` command)
- `controller` node uses two P controllers to minimize linear and angular error. The process is described in `workspace/limo_control/src/controller.cpp`.
- Utility classes/structs are put in util.cpp and can easily be expanded
- `app.sh` modified to launch the simulation, build and run the controller node, and specify an example target pose after starting the Docker container.

### Results
- Run `scripts/build/sim.sh` then `scripts/deploy/start.sh` to see a demo with an example target pose similar to the one in the given example video.
- See `demo.mp4` for a recording of what happens when the above is run.
- See `plot/plot.png` for a plot of linear and angular error to confirm that they are reduced into the requested threshold. The top graph shows the errors across the entire time range, and the bottom graph shows a zoomed-in version where the thresholds are clearly visible.
- Odometry data and the position reported in Gazebo do not match perfectly. I had to tune the bot track width in `workspace/limo_simulation/urdf/limo_four_diff.gazebo` on line 11 to get acceptable results. Also, the simulation was not starting the bot at the origin, so I reset those values. No other changes have been made to the simulation setup.

### Other Notes
- If trying yourself, remember to `chmod +x app.sh` locally before running `start.sh`
- Dockerfile was modified slightly to improve build times when only source files change, so that dependencies don't need to be reinstalled