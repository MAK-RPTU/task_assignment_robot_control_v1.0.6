#include <iostream>
#include "simulation.hpp"
#include "controller.hpp"
#include <cassert>

int main() {
    constexpr double dt = 1e-3;
    Eigen::Matrix<double, 6, 1> q_home{0, -M_PI/2, M_PI/2, -M_PI/2, -M_PI/2, M_PI/2};
    std::string urdf_filename = PINOCCHIO_MODEL_DIR + std::string("/urdf/ur5_robot.urdf");

    simulation::system robot{urdf_filename, dt, q_home};

    control::PDGravityController pd_controller(q_home, 100.0, 30.0);
    control::SimplePIDController pid_controller(q_home, 100.0, 15.0, 30.0);

    Eigen::VectorXd tau_pd = pd_controller.compute_torque(robot, dt);
    Eigen::VectorXd tau_pid = pid_controller.compute_torque(robot, dt);

    assert(tau_pd.size() == q_home.size());
    assert(tau_pid.size() == q_home.size());

    std::cout << "Test passed: Controllers produce correct torque vector sizes.\n";
    return 0;
}
