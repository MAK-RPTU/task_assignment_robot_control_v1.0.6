#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include "convenience_helpers.hpp"

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <Eigen/Core>

#include <string>

namespace simulation {
struct joint_states {
    joint_states(Eigen::Index const dof_in)
        : dof{dof_in}
        , q{Eigen::VectorXd::Zero(dof)}
        , dq{Eigen::VectorXd::Zero(dof)}
    {
    }

    joint_states(Eigen::VectorXd const& q_in)
        : dof{q_in.size()}
        , q{q_in}
        , dq{Eigen::VectorXd::Zero(dof)}
    {
    }

    joint_states(Eigen::VectorXd const& q_in, Eigen::VectorXd const& dq_in)
        : dof{q_in.size()}
        , q{q_in}
        , dq{dq_in}
    {
    }

    Eigen::Index dof;
    Eigen::VectorXd q;
    Eigen::VectorXd dq;
};

class system
{
public:
    system(std::string const& urdf_filename_in, double const dt_in)
        : urdf_filename{urdf_filename_in}
        , dt{dt_in}
        , model{}
        , dof{buildModel_and_get_dof()}
        , data{pinocchio::Data{model}}
        , state{simulation::joint_states(dof)}
    {
        update_data_object();
    }
    system(std::string const& urdf_filename_in, double const dt_in, Eigen::VectorXd const& q_init)
        : urdf_filename{urdf_filename_in}
        , dt{dt_in}
        , model{}
        , dof{buildModel_and_get_dof()}
        , data{pinocchio::Data{model}}
        , state{simulation::joint_states(q_init)}
    {
        update_data_object();
    }
    system(const system&)            = default;
    system(system&&)                 = default;
    system& operator=(const system&) = delete;
    system& operator=(system&&)      = delete;

    inline auto getJacobian() -> Eigen::MatrixXd  // joint jacobian matrix of TCP wrt. inertial/world frame
    {
        pinocchio::Data::Matrix6x J(6, dof);
        J.setZero();
        pinocchio::getJointJacobian(model, data, dof, pinocchio::ReferenceFrame::WORLD, J);
        return J.rightCols(6);
    };

    inline auto getJacobianDerivative()
        -> Eigen::MatrixXd  // time derivative of joint jacobian matrix of TCP wrt. inertial/world frame
    {
        pinocchio::Data::Matrix6x dJ(6, dof);
        dJ.setZero();
        pinocchio::getJointJacobianTimeVariation(model, data, dof, pinocchio::ReferenceFrame::WORLD, dJ);
        return data.dJ;
    };

    // Dynamic effects
    inline auto getGravityVector() -> Eigen::VectorXd { return data.g; };
    inline auto getCoriolisMatrix() -> Eigen::MatrixXd { return data.C; };
    inline auto getNonlinearEffects() -> Eigen::VectorXd { return data.nle; }
    // Note: getNonlinearEffects returns a vector that includes Coriolis, centrifugal and gravitational effects

    inline auto getMassMatrix() -> Eigen::MatrixXd { return data.M; };        // joint space inertia/mass matrix
    inline auto getMassmatrixInv() -> Eigen::MatrixXd { return data.Minv; };  // inverted inertia/mass matrix
    auto forwardKinematics(Eigen::VectorXd const& q, std::string const& frame = "tool0")
        -> pinocchio::SE3;  // get cartesian pose of a given frame (default: TCP)

    void apply_torque(Eigen::VectorXd const& tau, Eigen::Vector3d const& I_f_ext_tcp = Eigen::Vector3d::Zero());

private:
    int buildModel_and_get_dof();  // helper function for using initializer list in constructors
    void update_data_object();     // update the pinocchio data object

    /*
     * member variables (order is important for initializer list in constructors!)
     */
private:
    std::string const urdf_filename;
    double const dt;         // sample time
    pinocchio::Model model;  // pinocchio model object, containing all constant parameters
    Eigen::Index const dof;  // kinematic degrees of freedom
    pinocchio::Data data;    // pinoccho data object, containing all computed values

public:
    simulation::joint_states state;
};

}  // namespace simulation
#endif
