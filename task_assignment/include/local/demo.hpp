#ifndef DEMO_HPP
#define DEMO_HPP

#include "controller.hpp"
#include "simulation.hpp"

namespace demo {

// Base demo interface
class Demo {
public:
    virtual ~Demo() = default;
    virtual void run(simulation::system& robot,
                     control::Controller& controller,
                     double t_final, double dt) = 0;
};

// Home demo
class HomeDemo : public Demo {
public:
    void run(simulation::system& robot,
             control::Controller& controller,
             double t_final, double dt) override;
};

// External force demo
class ExternalForceDemo : public Demo {
public:
    ExternalForceDemo(double t_ext_force, double duration);
    void run(simulation::system& robot,
             control::Controller& controller,
             double t_final, double dt) override;
private:
    double t_ext_force_;
    double duration_;
};

// Trajectory demo
class TrajectoryDemo : public Demo {
public:
    TrajectoryDemo(Eigen::VectorXd q_start, Eigen::VectorXd q_end);
    void run(simulation::system& robot,
             control::Controller& controller,
             double t_final, double dt) override;
private:
    Eigen::VectorXd q_start_;
    Eigen::VectorXd q_end_;
};

} // namespace demo

#endif
