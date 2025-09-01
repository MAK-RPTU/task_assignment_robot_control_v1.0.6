
#include "simulation.hpp"

#include <pinocchio/algorithm/aba-derivatives.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/force.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

namespace simulation {

void system::apply_torque(Eigen::VectorXd const& tau, Eigen::Vector3d const& I_f_ext_tcp)
{
    // transform the external force from the the initial frame to the lst joint 6 (not the tcp!)
    Eigen::Matrix3d const R_IE   = forwardKinematics(state.q, "wrist_3_link").rotation();
    Eigen::Matrix3d const R_EI   = R_IE.transpose();
    Eigen::VectorXd J6_f_ext_tcp = Eigen::VectorXd::Zero(6);
    J6_f_ext_tcp.topRows(3)      = R_EI * I_f_ext_tcp;

    // plug together all joint-based external forces, and apply the provided external force to the last joint
    typedef pinocchio::ForceTpl<double, 0> Force;
    typedef pinocchio::container::aligned_vector<Force> AlignedVector;
    AlignedVector f_ext_all(model.njoints, pinocchio::Force::Zero());
    f_ext_all.back() = pinocchio::Force::Vector6(J6_f_ext_tcp);

    // compute forward dynamics
    Eigen::VectorXd const ddq = pinocchio::aba(model, data, state.q, state.dq, tau, f_ext_all);

    state.q  = pinocchio::integrate(model, state.q, state.dq * dt);
    state.dq = pinocchio::integrate(model, state.dq, ddq * dt);

    update_data_object();
}

void system::update_data_object()
{
    pinocchio::computeAllTerms(model, data, state.q, state.dq);
    pinocchio::updateFramePlacements(model, data);
}

int system::buildModel_and_get_dof()
{
    pinocchio::urdf::buildModel(urdf_filename, model);
    int const dof = model.nq;

    std::cout << "model name: " << model.name << "(" << dof << " dof)" << std::endl;

    return dof;
}

auto system::forwardKinematics(Eigen::VectorXd const& q, std::string const& frame) -> pinocchio::SE3
{
    static auto local_data = pinocchio::Data{model};  // to not corrupt system member variable `data`
    pinocchio::framesForwardKinematics(model, local_data, q);

    int const end_effector_id     = model.getFrameId(frame);
    pinocchio::SE3 const TCP_pose = local_data.oMf[end_effector_id];

    return TCP_pose;
}
}  // namespace simulation
