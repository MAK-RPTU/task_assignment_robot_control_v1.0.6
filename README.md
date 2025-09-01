# task_assignment_robot_control_v1.0.6
Task Assignment: Robot Control for a UR5 Manipulator A C++ project, implementing a PD controller to stabilize a UR5 robot in a Bullet Physics simulation. Includes a demonstration of external force disturbance rejection.

##

- Go to the task_assignment directory:

    `cd task_assignment_robot_control_v1.0.6/task_assignment/`

- Build the docker image:

    `docker build -t meeran_assignment .`


1. Execute below command to mount the local directory contents (folders and files) to docker assignment directory. Specify yout image and tag name

    `docker run -it --name assignment -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw meeran_assignment`

### Explanation:

- --name assignment - assigns name to the container
- -e DISPLAY=$DISPLAY passes your current display ID (:0 usually).

- -v /tmp/.X11-unix:/tmp/.X11-unix → mounts the X11 socket so container apps can talk to your X server.

- :rw makes sure it can read/write.

2. Inside the `assignment` directory in docker execute below commands step by step:

    `cmake -S . -B build`

    `cmake --build build`

3. IN order to avoid dependencies conflict export the path while remaining in the `assignment` directory

`export LD_LIBRARY_PATH="$(pwd)/lib:$LD_LIBRARY_PATH"`

4. Then run the robot_controller_assignment file using below command and it will give you joint positions

`./build/src/robot_controller_assignment `

5. A basic visualization for robot

`python3 ./src/basic_visualize.py`


## Reference:

- Git repository:

    `git clone https://github.com/MAK-RPTU/task_assignment_robot_control_v1.0.6.git`