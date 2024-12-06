#include "app/implementations/screwskinematicssolver.h"

#include <utility/math.h>

using namespace AIS4104;

ScrewsKinematicsSolver::ScrewsKinematicsSolver(Eigen::Matrix4d m, std::vector<Eigen::VectorXd> screws, Simulation::JointLimits limits)
    : ScrewsKinematicsSolver(std::move(m), std::move(screws), 4.e-3, 4.e-3, std::move(limits))
{
}

ScrewsKinematicsSolver::ScrewsKinematicsSolver(Eigen::Matrix4d m, std::vector<Eigen::VectorXd> space_screws, double v_e, double w_e, Simulation::JointLimits limits)
    : KinematicsSolver(std::move(limits))
    , m_ve(v_e)
    , m_we(w_e)
    , m_m(std::move(m))
    , m_screws(std::move(space_screws))
{
}

void ScrewsKinematicsSolver::set_epsilons(double v_e, double w_e)
{
    m_ve = v_e;
    m_we = w_e;
}

uint32_t ScrewsKinematicsSolver::joint_count() const
{
    return m_screws.size();
}

Eigen::Matrix4d ScrewsKinematicsSolver::fk_solve(const Eigen::VectorXd &joint_positions)
{
    //Parameters for KUKA KR 6 R900 sixx Agilus
    double Z_1 = 0.400; double X_1 = 0.025; //meters
    double X_2 = 0.455; //meters
    double Z_3 = 0.035; //meters
    double X_4 = 0.420; //meters
    double X_5 = 0.08; //meters

    double t_01 = joint_positions[0]; double t_12 = joint_positions[1]; double t_23 = joint_positions[2];
    double t_34 = joint_positions[3]; double t_45 = joint_positions[4]; double t_56 = joint_positions[5];

    Eigen::Vector3d w_1{0.0,0.0,1.0};
    Eigen::Vector3d w_2{0.0,1.0,0.0};
    Eigen::Vector3d w_3{0.0,1.0,0.0};
    Eigen::Vector3d w_4{1.0,0.0,0.0};
    Eigen::Vector3d w_5{0.0,1.0,0.0};
    Eigen::Vector3d w_6{1.0,0.0,0.0};

    Eigen::Vector3d q_01{0.0,0.0,0.0};
    Eigen::Vector3d q_02{X_1,0.0,Z_1};
    Eigen::Vector3d q_03{X_1+X_2,0.0,Z_1};
    Eigen::Vector3d q_04{X_1+X_2,0.0,Z_1+Z_3};
    Eigen::Vector3d q_05{X_1+X_2+X_4,0.0,Z_1+Z_3};
    Eigen::Vector3d q_06{X_1+X_2+X_4+X_5,0.0,Z_1+Z_3};

    Eigen::Matrix3d R_sb = Eigen::Matrix3d::Identity();
    Eigen::Vector3d q_sb{X_1+X_2+X_4+X_5,0.0,Z_1+Z_3};

    //Equation without reference, velocity formula, page 101, MR pre-print 2019
    Eigen::Matrix4d S_01 = utility::matrix_exponential(w_1,-w_1.cross(q_01),t_01);
    Eigen::Matrix4d S_02 = utility::matrix_exponential(w_2,-w_2.cross(q_02),t_12);
    Eigen::Matrix4d S_03 = utility::matrix_exponential(w_3,-w_3.cross(q_03),t_23);
    Eigen::Matrix4d S_04 = utility::matrix_exponential(w_4,-w_4.cross(q_04),t_34);
    Eigen::Matrix4d S_05 = utility::matrix_exponential(w_5,-w_5.cross(q_05),t_45);
    Eigen::Matrix4d S_06 = utility::matrix_exponential(w_6,-w_6.cross(q_06),t_56);

    //Equation (4.13), page 140, MR pre-print 2019
    Eigen::Matrix4d T = S_01*S_02*S_03*S_04*S_05*S_06*utility::transformation_matrix(R_sb,q_sb);
    return T;
}

Eigen::VectorXd ScrewsKinematicsSolver::ik_solve(const Eigen::Matrix4d &t_sd, const Eigen::VectorXd &j0)
{
    return ik_solve(t_sd, j0, [&](const std::vector<Eigen::VectorXd> &) { return 0u; });
}

//TASK: Implement ik_solve using screws.
Eigen::VectorXd ScrewsKinematicsSolver::ik_solve(const Eigen::Matrix4d &t_sd, const Eigen::VectorXd &j0, const std::function<uint32_t(const std::vector<Eigen::VectorXd> &)> &solution_selector)
{
    return Eigen::VectorXd::Zero(j0.size());
}

std::pair<Eigen::Matrix4d, std::vector<Eigen::VectorXd>> ScrewsKinematicsSolver::space_chain()
{
    return {m_m, m_screws};
}

//TASK: Implement body_chain() using space_chain().
std::pair<Eigen::Matrix4d, std::vector<Eigen::VectorXd>> ScrewsKinematicsSolver::body_chain()
{
    return space_chain();
}

//TASK: Implement space_jacobian() using space_chain()
Eigen::MatrixXd ScrewsKinematicsSolver::space_jacobian(const Eigen::VectorXd &current_joint_positions)
{
    return Eigen::MatrixXd::Identity(current_joint_positions.size(), current_joint_positions.size());
}

//TASK: Implement body_jacobian() using body_chain()
Eigen::MatrixXd ScrewsKinematicsSolver::body_jacobian(const Eigen::VectorXd &current_joint_positions)
{
    return Eigen::MatrixXd::Identity(current_joint_positions.size(), current_joint_positions.size());
}
