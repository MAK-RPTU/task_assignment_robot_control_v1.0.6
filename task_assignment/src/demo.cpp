#include "demo.hpp"
#include <fstream>
#include <iostream>

namespace demo {

// ------------------ Home Demo ------------------
void HomeDemo::run(simulation::system& robot,
                   control::Controller& controller,
                   double t_final, double dt) {
    std::ofstream file("log_home.txt");
    for (double t = 0; t <= t_final; t += dt) {
        Eigen::VectorXd tau = controller.compute_torque(robot, dt);
        robot.apply_torque(tau);
        file << "time:\t" << t
             << "\tq:\t" << robot.state.q.transpose()
             << "\t dq:\t" << robot.state.dq.transpose()
             << std::endl;
    }
    std::cout << "Final state: " << robot.state.q.transpose() << std::endl;
    std::cout << "Log written to log_home.txt\n";
}

// ------------------ External Force Demo ------------------
ExternalForceDemo::ExternalForceDemo(double t_ext_force, double duration)
    : t_ext_force_(t_ext_force), duration_(duration) {}

void ExternalForceDemo::run(simulation::system& robot,
                            control::Controller& controller,
                            double t_final, double dt) {
    std::ofstream file("log_ext_force.txt");
    for (double t = 0; t <= t_final; t += dt) {
        Eigen::VectorXd tau = controller.compute_torque(robot, dt);
        if (t < t_ext_force_) {
            robot.apply_torque(tau);
        } else if (t < t_ext_force_ + duration_) {
            Eigen::Vector3d I_f_ext{0, 0, -50};
            robot.apply_torque(tau, I_f_ext);
        } else {
            robot.apply_torque(tau);
        }
        file << "time:\t" << t
             << "\tq:\t" << robot.state.q.transpose()
             << "\t dq:\t" << robot.state.dq.transpose()
             << std::endl;
    }
    std::cout << "Final state: " << robot.state.q.transpose() << std::endl;
    std::cout << "Log written to log_ext_force.txt\n";
}

// ------------------ Trajectory Demo ------------------
TrajectoryDemo::TrajectoryDemo(Eigen::VectorXd q_start, Eigen::VectorXd q_end)
    : q_start_(std::move(q_start)), q_end_(std::move(q_end)) {}

void TrajectoryDemo::run(simulation::system& robot,
                         control::Controller& controller,
                         double t_final, double dt) {
    std::ofstream file("log_trajectory.txt");
    for (double t = 0; t <= t_final; t += dt) {
        double alpha = t / t_final;
        Eigen::VectorXd q_desired =
            (1.0 - alpha) * q_start_ + alpha * q_end_;
        controller.setDesired(q_desired);
        robot.apply_torque(controller.compute_torque(robot, dt));
        file << "time:\t" << t
             << "\tq:\t" << robot.state.q.transpose()
             << "\t dq:\t" << robot.state.dq.transpose()
             << std::endl;
    }
    std::cout << "Final state: " << robot.state.q.transpose() << std::endl;
    std::cout << "Log written to log_trajectory.txt\n";
}

// Extend with more demos as needed

} // namespace demo
