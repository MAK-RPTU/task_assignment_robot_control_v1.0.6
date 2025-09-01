# robot control task assignment v1.0.5 [2025-08-11]
Provided is a simulation framework of a serial 6-dof robot. It loads a robot description file (urdf) and simulates its dynamic evolution over time, with joint torques `tau` as system inputs and the robot state `robot.state`, consisting of joint positions `q` and resp velocities `dq`, as available system outputs. After `t_ext_force` seconds, an external force is applied to the TCP.

Your implementation of the tasks mentioned below, will serve as basis for a technical discussion during the technical interview.

## Prerequisites
- `cmake 3.16+`
- `gcc 8.4+` or `clang 9.0+`
- optional (cf. Tips below): `python 3.8+` with `pip 21.0+` (older versions might not provide the required module versions listed in `requirements.txt`)

## Provided files
- `CMakeLists.txt` -> compiles the entire project
- `src/main.cpp` -> contains the `main()` function that runs the simulation
- `include/local/simulation.hpp` and `src/simulation.cpp` -> provides an interface to pinocchio for dynamic simulations. 
   You can, but don't have to touch these files at all. Just have a look at the interface defined in `include/local/simulation.hpp`
- `lib/` and `include/`-> provides shared libraries 
  - [`eigen 3.5.0`](https://eigen.tuxfamily.org/) for linear algebra
  - [`pinocchio`](https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html) for rigid body dynamics (uses `eigen` internally).
  - `boost` is a dependency of `pinocchio`
  - other system libraries that might be missing on your system
- `share/robot_model/` -> includes the urdf of our robot, and also the meshes for 3d visualization

### optional
- `requirements.txt` -> list of relevant python pip packages (cf. below)
- `src/visualize3d.py` -> a custom python module that uses `pybullet` for convenient 3d visualization of robot poses (its use is optional, see usage example below)
- `log.txt` -> is created when running the built executable

## build instructions
- run the following in the top-level of the task assignment folder
    ```
    > cmake -S . -B build
    > cmake --build build
    ```
- this creates a binary `build/src/robot_controller_assignment` (the initial build takes around 3min)
- running this binary prints the final joint pose and creates a `log.txt` file with the dynamic evolution of the simulation, for analysis and 3d visualization (cf. below)

### in case of issues
- HINT: in case you are missing some shared libraries on your system when executing the binary, consider to call a `export LD_LIBRARY_PATH="$(pwd)/lib:$LD_LIBRARY_PATH"` from the top-level directory
- HINT: if you still face issues, consider working in a docker (cf. this repo was tested with `Dockerfile`)

## Assignment Tasks

1. **Build and Run**: Ensure you can successfully compile and run the provided CMake project. The simulation should load the robot URDF and execute its dynamic evolution. If you encounter issues you cannot resolve, please reach out using the contact information below.
2. **Robot Control**: Implement a controller to stabilize the robot in its home pose.
3. **Demonstration**: Extend the basic functionality by creating a robot demo of your choice. This could involve trajectory tracking, interaction with external forces, or any other feature that showcases your approach.

### Guidelines
- You are encouraged to modify or improve any part of the codebase as needed.
- Direct interaction with `pinocchio` is optional; for standard controllers, the interfaces in `include/local/simulation.hpp` (using `Eigen`) should be sufficient.
- Structure your code and submission as you would for a professional merge/pull request intended for production.


## Tips
- the `requirements` file lists relevant python packages that you can use (this is optional). 
  Note: tested with `python 3.8` and `pip 21.0` 
  ```
  pip3 install --upgrade pip
  python3 -m pip install -r requirements.txt
  ```
- for rapid prototyping, you can also make use the pinocchio python api `pin` (cf. `requirements.txt`)
- we provided an easy 3d visualization setup based on the python package `pybullet` (cf. `requirements.txt`). Minimal working example:
    ```
    import numpy as np
    import pybullet
    import time
    import visualize3d

    viz = visualize3d.gui()
    while True:
      q = np.random.uniform(-np.pi/2, np.pi/2, 6)
      viz.set_joints(q)
      time.sleep(1)
    ```

## FAQ
In case any questions or problems arise, please get in touch (email, or e.g. whatsapp):
- gerold.huber@agile-robots.com, +49 174 3707218

## Copyright
Copyright © 2024 Agile Robots AG. All right reserved.