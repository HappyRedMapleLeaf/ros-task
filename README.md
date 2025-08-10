# Task Description

Please read through [this](https://www.overleaf.com/read/dmgrrcmpkbkq#211e69) document before moving forward.

### Software Structure
```
- docker -- Where the Dockerfile lives.
- scripts -- Where necessary external scripts live.
- workspace -- Where all the packages live.
```

### Build the simulator

```bash
./scripts/build/sim.sh
```

### Run the simulator

```bash
./scripts/deploy/devel.sh # To enter the docker container
ros2 launch limo_simulation limo.launch.py # To launch the simulator
docker exec -it limo_bot /bin/bash # To enter the container after starting

# Run controller node manually:
cd /root/workspace
colcon build
ros2 run limo_control controller

# Update target pose:
ros2 topic pub /target geometry_msgs/msg/Pose2D "{x: 10.0, y: 10.0, theta: 1.571}" --once

./scripts/deploy/start.sh # To run the whole demo
./scripts/deploy/stop.sh # From another terminal, if things get stuck. Ctrl+C should work for start.sh though.
```

### What do I edit?

1. Modify the package `limo_control` in the workspace directory for adding your c++ controller program.
2. Make a launch file that can launch everything (Controller and Simualation).
3. Modify `scripts/deploy/app.sh` such that, when `scripts/deploy/start.sh` is run, the task is executed automatically.

### Known Issues

1. This will not work with docker desktop, please do not use it, use the default engine.

Feel free to modify anything else if it does not work as expected.