#include <iostream>
#include <Eigen/Core>
#include "simulation.hpp"
#include "controller.hpp"
#include "demo.hpp"
#include <cassert>

int main() {
    constexpr double dt = 1e-3;

    Eigen::Matrix<double, 6, 1> q_zero{Eigen::VectorXd::Zero(6)};
    Eigen::Matrix<double, 6, 1> q_home{0, -M_PI / 2, M_PI / 2, -M_PI / 2, -M_PI / 2, M_PI / 2};
    Eigen::Matrix<double, 6, 1> q_desired{0.5, -1.0, 1.2, -0.7, 0.3, 0.8};

    std::string urdf_filename =
        PINOCCHIO_MODEL_DIR + std::string("/urdf/ur5_robot.urdf");

    // Create robot
    simulation::system robot{urdf_filename, dt, q_home};

    // Controllers
    control::PDGravityController pd_controller(q_home, 100.0, 30.0);
    control::SimplePIDController pid_controller(q_home, 100.0, 15.0, 30.0);

    // Test 1: Torque vector size check
    Eigen::VectorXd tau_pd = pd_controller.compute_torque(robot, dt);
    Eigen::VectorXd tau_pid = pid_controller.compute_torque(robot, dt);

    assert(tau_pd.size() == q_home.size()+1);
    assert(tau_pid.size() == q_home.size());
    std::cout << "Test 1 passed: Torque vectors have correct size.\n";

    // Test 2: Robot state update
    robot.apply_torque(tau_pd);
    assert(!robot.state.q.hasNaN());
    std::cout << "Test 2 passed: Robot state updated successfully with PD controller.\n";

    robot.apply_torque(tau_pid);
    assert(!robot.state.q.hasNaN());
    std::cout << "Test 3 passed: Robot state updated successfully with PID controller.\n";

    // Test 3: TrajectoryDemo minimal check (without writing files)
    demo::TrajectoryDemo traj_demo(q_home, q_desired);

    double t_final = 0.01;  // very short simulation for test
    traj_demo.run(robot, pd_controller, t_final, dt);
    std::cout << "Test 4 passed: TrajectoryDemo ran without errors.\n";

    return 0;
}
