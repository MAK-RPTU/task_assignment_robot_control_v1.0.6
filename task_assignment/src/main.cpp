#include "simulation.hpp"
#include "controller.hpp"
#include "demo.hpp"

#define M_PI 3.14159265358979323846

int main() {
    // Simulation parameters
    constexpr double dt = 1e-3;
    constexpr double t_final = 5.0;

    std::string urdf_filename =
        PINOCCHIO_MODEL_DIR + std::string("/urdf/ur5_robot.urdf");

    Eigen::Matrix<double, 6, 1> q_zero{Eigen::VectorXd::Zero(6)};

    // Home configuration
    Eigen::Matrix<double, 6, 1> q_home{0, -M_PI / 2, M_PI / 2,
                                       -M_PI / 2, -M_PI / 2, M_PI / 2};

    // Desired configuration for trajectory demo
    Eigen::Matrix<double, 6, 1> q_desired{0.5, -1.0, 1.2, -0.7, 0.3, 0.8};

    // Create robot
    simulation::system robot{urdf_filename, dt, q_home};

    // Controllers
    // PD + Gravity
    control::PDGravityController pd_controller(q_home, 100.0, 30.0);
    // Simple PID (no gravity compensation)
    control::SimplePIDController pid_controller(q_home, 100.0, 15.0, 30.0);

    // Choose controller
    // Pointer to controller (Comment/uncomment to switch controller)
    // control::Controller* controller = &pid_controller;
    control::Controller* controller = &pd_controller;  // switch to PD + Gravity

    // Choose demo (try HomeDemo first)
    // demo::HomeDemo d; // home position demo
    // demo::ExternalForceDemo d(3.0, 0.5);  // external force demo
    demo::TrajectoryDemo d(q_home, q_desired); //Trajectory demo
    // Further demos can be 

    // Run demo
    d.run(robot, *controller, t_final, dt);

    return 0;
}
