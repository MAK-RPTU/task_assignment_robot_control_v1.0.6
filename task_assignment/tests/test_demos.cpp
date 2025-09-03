#include <iostream>
#include "simulation.hpp"
#include "controller.hpp"
#include "demo.hpp"
#include <cassert>

int main() {
    constexpr double dt = 1e-3;
    double t_final = 0.01; // short run for test
    Eigen::Matrix<double, 6, 1> q_home{0, -M_PI/2, M_PI/2, -M_PI/2, -M_PI/2, M_PI/2};
    Eigen::Matrix<double, 6, 1> q_desired{0.5, -1.0, 1.2, -0.7, 0.3, 0.8};
    std::string urdf_filename = PINOCCHIO_MODEL_DIR + std::string("/urdf/ur5_robot.urdf");

    simulation::system robot{urdf_filename, dt, q_home};
    control::PDGravityController pd_controller(q_home, 100.0, 30.0);

    // Trajectory demo minimal run
    demo::TrajectoryDemo traj_demo(q_home, q_desired);
    traj_demo.run(robot, pd_controller, t_final, dt);

    std::cout << "Test passed: TrajectoryDemo ran without crashing.\n";
    return 0;
}
