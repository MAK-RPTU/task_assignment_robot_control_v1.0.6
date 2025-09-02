# task_assignment_robot_control_v1.0.6

This project implements **robot control for a UR5 manipulator** in a Bullet Physics simulation.  
It includes:

- A **PD + Gravity Compensation Controller** and a **PID Controller**  
- Demonstrations of:  
  - **Stabilizing the UR5 at home position**  
  - **Trajectory tracking between two configurations**  
  - **External force disturbance rejection**  
- A **basic Python visualization** tool for log playback  

---

## 🎬 Quick Demo Showcase

| Demo Name           | Command to Run                         | Log File                 | Notes                             |
|--------------------|----------------------------------------|--------------------------|----------------------------------|
| Home Position       | `./robot_controller_assignment`        | `log_home.txt`           | Holds the robot at the home pose |
| Trajectory Tracking | `./robot_controller_assignment`        | `log_trajectory.txt`     | Moves robot from start → end q   |
| External Force      | `./robot_controller_assignment`        | `log_ext_force.txt`      | Applies external disturbance     |

> Use the visualization script to play back any of the logs:  
> `python3 basic_visualize --log <log_file.txt>`

---

## 🚀 Getting Started

### 1. Clone the Repository
```bash
git clone https://github.com/MAK-RPTU/task_assignment_robot_control_v1.0.6.git -b develop
```

```bash
cd task_assignment_robot_control_v1.0.6/task_assignment/
```

##

- Clone the repo:
    `git clone https://github.com/MAK-RPTU/task_assignment_robot_control_v1.0.6.git -b develop`

- Go to the task_assignment directory:

    `cd task_assignment_robot_control_v1.0.6/task_assignment/`

- Build the docker image:

    `docker build -t meeran_assignment .`

1. Execute below command to mount the local directory contents (folders and files) to docker assignment directory. Specify your image and tag name

    e.g: `docker run -it --name assignment -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v $(pwd):/assignment meeran_assignment:latest`

### Explanation:

- --name - assigns name to the container
- -e DISPLAY=$DISPLAY passes your current display ID (:0 usually).

- -v /tmp/.X11-unix:/tmp/.X11-unix - mounts the X11 socket so container apps can talk to your X server from bash terminal.

- :rw makes sure it can read/write.

2. Inside the `assignment` directory in docker execute below commands step by step:

    `cmake -S . -B build`

    `cmake --build build`

3. IN order to avoid dependencies conflict export the path while remaining in the `assignment` directory

`export LD_LIBRARY_PATH="$(pwd)/lib:$LD_LIBRARY_PATH"`

4. Then run the robot_controller_assignment file using below command and it will log joint positions in a txt file. Here is a symbolic link to the `./build/src/robot_controller_assignment ` for easier CLI.

`./robot_controller_assignment`

5. A basic visualization for robot

    ### Method1 (simple)
    If using vscode with running Dev container then simply run below commands to visualize:

    `python3 basic_visualize` By default it will go to home position and retain there

    If we specify argument to our specific log file then it would visualize the respective demo e.g. Provided sample demo txt files name as log_ext_force.txt, log_home, log_trajectory.

    `python3 basic_visualize --log <log.txt>` 

    e.g:

    `python3 basic_visualize --log log_ext_force.txt` 

    ### Method2
    If want to run via docker root terminal open a separate local terminal and allow the root user to access the running X server. The current X server is indicated by the DISPLAY environment variable.

    `xhost +local:root`

    and then run the below command inside docker bash shell and also can run the demos as stated above:

    `python3 basic_visualize`


## Reference:

- Git repository:

    `git clone https://github.com/MAK-RPTU/task_assignment_robot_control_v1.0.6.git -b develop`