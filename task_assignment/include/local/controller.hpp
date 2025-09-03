#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <Eigen/Core>
#include "simulation.hpp"
#include <cassert>

namespace control {

// Base controller interface
class Controller {
public:
    virtual ~Controller() = default;
    virtual Eigen::VectorXd compute_torque(simulation::system& robot, double dt) = 0;
    // Add this virtual method
    virtual void setDesired(const Eigen::VectorXd&) {}
};

// ------------------ PD + Gravity Compensation Controller ------------------
class PDGravityController : public Controller {
public:
    void setDesired(const Eigen::VectorXd& q_desired) {
        q_desired_ = q_desired;
    }

    PDGravityController(Eigen::VectorXd q_desired, double Kp, double Kd)
        : q_desired_(std::move(q_desired)), Kp_(Kp), Kd_(Kd) {}

    Eigen::VectorXd compute_torque(simulation::system& robot, double /*dt*/) override {
        assert(q_desired_.size() == robot.state.q.size() && "q_desired_ size must match robot DOF");

        Eigen::VectorXd q_error = q_desired_ - robot.state.q;
        Eigen::VectorXd dq_error = -robot.state.dq;
        return Kp_ * q_error + Kd_ * dq_error + robot.getGravityVector();
    }

private:
    Eigen::VectorXd q_desired_;
    double Kp_;
    double Kd_;
};

// ------------------ Simple PID Controller (No Gravity Compensation) ------------------
class SimplePIDController : public Controller {
public:
    SimplePIDController(Eigen::VectorXd q_desired, double Kp, double Ki, double Kd)
        : q_desired_(std::move(q_desired)), Kp_(Kp), Ki_(Ki), Kd_(Kd) {}

    Eigen::VectorXd compute_torque(simulation::system& robot, double dt) override {
        // Make sure integral vector matches robot DOF
        if (integral_.size() != robot.state.q.size()) {
            integral_ = Eigen::VectorXd::Zero(robot.state.q.size());
        }

        // If q_desired_ size mismatch, truncate or pad with zeros
        Eigen::VectorXd q_des = q_desired_;
        if (q_des.size() != robot.state.q.size()) {
            q_des.conservativeResize(robot.state.q.size());
            q_des.tail(robot.state.q.size() - q_des.size()).setZero();
        }

        Eigen::VectorXd q_error = q_des - robot.state.q;
        Eigen::VectorXd dq_error = -robot.state.dq;

        integral_ += q_error * dt;

        return Kp_ * q_error + Ki_ * integral_ + Kd_ * dq_error;
    }

private:
    Eigen::VectorXd q_desired_;
    Eigen::VectorXd integral_;
    double Kp_;
    double Ki_;
    double Kd_;
};

// Extend with more controllers as needed

} // namespace control

#endif
