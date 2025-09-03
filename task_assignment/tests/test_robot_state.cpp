#include <iostream>
#include <Eigen/Core>
#include "simulation.hpp"
#include <cassert>

int main() {
    constexpr double dt = 1e-3;
    Eigen::Matrix<double, 6, 1> q_home{0, -M_PI/2, M_PI/2, -M_PI/2, -M_PI/2, M_PI/2};
    std::string urdf_filename = PINOCCHIO_MODEL_DIR + std::string("/urdf/ur5_robot.urdf");

    simulation::system robot{urdf_filename, dt, q_home};

    // Basic robot state check
    assert(robot.state.q.size() + 1 == 6);
    assert(robot.state.dq.size() == 6);
    std::cout << "Test passed: Robot state initialized correctly.\n";

    return 0;
}
