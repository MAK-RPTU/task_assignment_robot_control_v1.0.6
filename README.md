# task_assignment_robot_control_v1.0.6

| Test            | Status |
|-----------------|--------|
| Build     | ![Robot State](https://github.com/MAK-RPTU/task_assignment_robot_control_v1.0.6/actions/workflows/build-docker.yml/badge.svg?branch=develop) |
| Robot State     | ![Robot State](https://github.com/MAK-RPTU/task_assignment_robot_control_v1.0.6/actions/workflows/test_robot_state.yml/badge.svg?branch=develop) |
| Controllers     | ![Controllers](https://github.com/MAK-RPTU/task_assignment_robot_control_v1.0.6/actions/workflows/test_controllers.yml/badge.svg?branch=develop) |
| Demos           | ![Demos](https://github.com/MAK-RPTU/task_assignment_robot_control_v1.0.6/actions/workflows/test_demos.yml/badge.svg?branch=develop) |


This project implements **robot control for a UR5 manipulator** in a Bullet Physics simulation.  
It includes:

- A **PD + Gravity Compensation Controller** and a **PID Controller**  
- Demonstrations of:  
  - **Stabilizing the UR5 at home position**  
  - **Trajectory tracking between two configurations**  
  - **External force disturbance rejection**  
- A **basic Python visualization** tool for log playback  

---

## Quick Demo Showcase

| Demo Name           | Command to Run                         | Log File                 | Notes                             |
|--------------------|----------------------------------------|--------------------------|----------------------------------|
| Home Position       | `./robot_controller_assignment`        | `log_home.txt`           | Holds the robot at the home pose |
| Trajectory Tracking | `./robot_controller_assignment`        | `log_trajectory.txt`     | Moves robot from start , end q   |
| External Force      | `./robot_controller_assignment`        | `log_ext_force.txt`      | Applies external disturbance     |

> After following steps mentioned in [1](#1-clone-the-repository) , [2](#2-build-with-docker) and [5](#5-visualization) use below python file to visualize  
> Use the visualization script to play back any of the logs:  
> e.g. `python3 basic_visualize --log <log_file.txt>`

---

## Getting Started

### 1. Clone the Repository
```bash
git clone https://github.com/MAK-RPTU/task_assignment_robot_control_v1.0.6.git -b develop
```
```bash
cd task_assignment_robot_control_v1.0.6/task_assignment/
```

### 2. Build with Docker

Build the Docker image:

```bash
docker build -t meeran_assignment .
```

Run the container:

```bash
docker run -it --name assignment -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v $(pwd):/assignment meeran_assignment:latest
```

### Flags explained:

- --name → Assigns a name to the container

- -e DISPLAY=$DISPLAY : Shares your X display

- -v /tmp/.X11-unix:/tmp/.X11-unix : Mounts the X11 socket so container apps can talk to X server from bash terminal.

- -v $(pwd):/assignment : Mounts your current repo

### 3. Building the Project

```bash
cmake -S . -B build
```

```bash
cmake --build build
```
Export library path to avoid dependency conflicts:

```bash
export LD_LIBRARY_PATH="$(pwd)/lib:$LD_LIBRARY_PATH"
```
### 4. Running the Controller

Run the executable:

```bash
./robot_controller_assignment
```

This generates log files such as:

- log_home.txt - Home position demo

- log_trajectory.txt - Trajectory demo

- log_ext_force.txt - External force demo

### 5. Visualization
#### Method 1: Simple (Recommended)

If using VSCode Dev Containers:

```bash
python3 basic_visualize
```

By default it runs home_demo.txt. To visualize a specific log file:

```bash
python3 basic_visualize --log log_trajectory.txt
```
```bash
python3 basic_visualize --log log_ext_force.txt
```

#### Method 2: Docker Root Session

From a local terminal:

```bash
xhost +local:root
```

Then inside the container:

```bash
python3 basic_visualize --log log_home.txt
```

### 6. Project Structure

```
task_assignment/
├── include/local/                  # Header files
│   ├── controller.hpp              # Controllers (PID, PD+Gravity)
│   ├── demo.hpp                    # Demo interfaces & implementations
│   └── simulation.hpp              # Robot system & simulation
├── src/                            # Source files
│   ├── main.cpp                    # Entry point (choose controller + demo)
│   ├── demo.cpp                    # Demo implementations
│   ├── visualize3d.py              # visualization class for pybullet
│   ├── simulation.cpp              
│   ├── CMakeLists.txt              
│   └── basic_visualize.py          # Python log visualization
├── tests/                          # Tests
│   ├── test_controllers.cpp        # Controller test
│   ├── test_demos.cpp              # Demos test
│   ├── test_robot_state.py         # Robot state joint angles and joint velocities
│   ├── test_robot.cpp              # All tests combined (Sample)
│   └── CMakeLists.txt              
├── CMakeLists.txt      
├── Dockerfile      
├── basic_visualize                 # Symbolic link to Python log visualization
└── robot_controller_assignment     # Symbolic link to run main.cpp
```
### 7. Features Summary

- Controllers

    - PD + Gravity Compensation

    - PID

- Demos

    - Home position holding

    - Trajectory tracking

    - External force disturbance

- Visualization with Python

### 9. Tests:

All tests are located in the `tests/` directory:

  -  `test_controllers.cpp` - Validates controller logic.

  -  `test_demos.cpp` - Runs demonstration scenarios.

  -  `test_robot_state.py` - Checks robot state (joint angles & velocities).

  -  `test_robot.cpp` - Combined sample test covering multiple components.

Note:  
These are simple assertion-based tests, not full testing frameworks like GoogleTest or PyTest. The goal is to keep them lightweight and to demonstrate.

#### Running Tests

##### Build the project in Debug mode (required for test symbols & assertions). Before build don't forget to be in `task_assignment/` directory:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
```

```bash
cmake --build build
```
##### Go to the build directory:

```bash
cd build/
```

##### Run a specific test with CTest:

```bash
ctest -R <test_name> --verbose
```

##### Example:

```bash
ctest -R test_robot_state --verbose
```

##### Automated Tests (CI/CD)

- Use GitHub Actions to run tests automatically on every push to the `develop` branch.

- Each test has its own status badge in this README.

- One test is intentionally left failing 😃 to demonstrate the testing and CI/CD pipeline in action.

You can check the latest results directly on the `develop` branch.

### 10. Extending the Project

To add new functionality:

- Create a new controller - extend control::Controller

- Create a new demo - extend demo::Demo and implement run()

- Add visualization logs - pass the new log file to basic_visualize.py

---

### References

- GitHub Repo:

    ```bash
    git clone https://github.com/MAK-RPTU/task_assignment_robot_control_v1.0.6.git -b develop
    ```

- Bullet Physics Engine

- Pinocchio Robotics Library