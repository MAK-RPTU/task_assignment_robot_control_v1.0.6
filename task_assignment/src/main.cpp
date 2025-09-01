// #include "../sample_solution/controller.hpp"
#include "simulation.hpp"

#include "Eigen/Core"

#include <iostream>
#include <fstream>
#include <filesystem>

#define M_PI 3.14159265358979323846

int main()
{
    // simulation parameters
    double constexpr dt          = 1e-3;  // sample time
    double constexpr t_ext_force = 3.0;   // start time of external force
    double constexpr t_final     = 5.0;   // total simulation time

    std::string const urdf_filename = PINOCCHIO_MODEL_DIR + std::string("/urdf/ur5_robot.urdf");
    constexpr char log_file_name[]  = "log.txt";

    // distinct robot poses
    Eigen::Matrix<double, 6, 1> const q_zero{Eigen::VectorXd::Zero(6)};
    Eigen::Matrix<double, 6, 1> const q_home{0, -M_PI / 2, M_PI / 2, -M_PI / 2, -M_PI / 2, M_PI / 2};

    // simulation
    auto robot = simulation::system{urdf_filename, dt, q_home};
    auto file  = std::ofstream{log_file_name};
    for (double t = 0; t <= t_final; t += dt) {

        // Eigen::VectorXd tau = Eigen::VectorXd::Zero(6);  // TODO: replace with your controller

        //PD Controller
        Eigen::VectorXd tau(6);
        Eigen::VectorXd q_error = q_home - robot.state.q;
        Eigen::VectorXd dq_error = -robot.state.dq;

        // gains
        // Tuning method used: Ziegler-Nichols method
        // INcrease Kp until the system oscillates, then increase Kd until oscillation vanishes
        double Kp = 100.0;
        double Kd = 30.0;

        // PD control + gravity compensation g(q)
        tau = Kp * q_error + Kd * dq_error + robot.getGravityVector();

        // PD control + non linear compensation ())
        // tau = Kp * q_error + Kd * dq_error + robot.getNonlinearEffects();


        // apply an external force after t_ext_force
        if (t < t_ext_force) {
            robot.apply_torque(tau);
        }

        // else {
        //     Eigen::Vector3d I_f_ext{0, 0, -50};  // external force wrt. to the inertial/world frame
        //     robot.apply_torque(tau, I_f_ext);
        // }

        else if (t < t_ext_force + 0.5) { // apply external force wrt. to the inertial/world frame for 0.5s
            Eigen::Vector3d I_f_ext{0, 0, -50};
            robot.apply_torque(tau, I_f_ext);
        }

        else {
            robot.apply_torque(tau); // back to normal PD+gravity control
        }
        // log time evolution of the simulation.
        // TODO: feel free to keep/extend/replace/delete this logging to your liking
        file << "time:\t" << t                             //
             << "\ttau:\t" << tau.transpose()              //
             << "\tstate:\t" << robot.state.q.transpose()  //
             << "\t|\t" << robot.state.dq.transpose()      //
             << std::endl;
    }

    std::cout << "final joint positions " << robot.state.q.transpose() << "\n";
    std::cout << "log written to " << log_file_name << "\n";
}
