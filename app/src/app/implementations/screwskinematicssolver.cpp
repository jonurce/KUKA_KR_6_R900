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

//DONE: Implement fk_solve using screws.
Eigen::Matrix4d ScrewsKinematicsSolver::fk_solve(const Eigen::VectorXd &joint_positions)
{
    //Parameters for KUKA KR 6 R900 sixx Agilus
    double Z_1 = 0.400; double X_1 = 0.025; //meters
    double X_2 = 0.455; //meters
    double Z_3 = 0.035; //meters
    double X_4 = 0.420; //meters
    double X_5 = 0.08; //meters

    //Equation (4.13) (c), page 140, MR pre-print 2019 -
    //(c) Joint variables theta_1,..., theta_n.
    double t_01 = joint_positions[0]; double t_12 = joint_positions[1]; double t_23 = joint_positions[2];
    double t_34 = joint_positions[3]; double t_45 = joint_positions[4]; double t_56 = joint_positions[5];

    //Equation (4.13) (a), page 140, MR pre-print 2019 -
    //(a) End effector configuration M when the robot is at its home position
    Eigen::Matrix3d R_sb = Eigen::Matrix3d::Identity();
    Eigen::Vector3d q_sb{X_1+X_2+X_4+X_5,0.0,Z_1+Z_3};

    //Equation (4.13) (b), page 140, MR pre-print 2019 -
    //(b) Screw axes S1,..., Sn expressed in the fixed base frame, corresponding
    //to the joint motions when the robot is at its home position
    //Equation without reference, velocity formula, page 101, MR pre-print 2019
    Eigen::Vector3d s_1{0.0,0.0,1.0};
    Eigen::Vector3d s_2{0.0,1.0,0.0};
    Eigen::Vector3d s_3{0.0,1.0,0.0};
    Eigen::Vector3d s_4{1.0,0.0,0.0};
    Eigen::Vector3d s_5{0.0,1.0,0.0};
    Eigen::Vector3d s_6{1.0,0.0,0.0};

    Eigen::Vector3d q_01{0.0,0.0,0.0};
    Eigen::Vector3d q_02{X_1,0.0,Z_1};
    Eigen::Vector3d q_03{X_1+X_2,0.0,Z_1};
    Eigen::Vector3d q_04{X_1+X_2,0.0,Z_1+Z_3};
    Eigen::Vector3d q_05{X_1+X_2+X_4,0.0,Z_1+Z_3};
    Eigen::Vector3d q_06{X_1+X_2+X_4+X_5,0.0,Z_1+Z_3};

    Eigen::Matrix4d S_01 = utility::matrix_exponential(s_1,-s_1.cross(q_01),t_01);
    Eigen::Matrix4d S_02 = utility::matrix_exponential(s_2,-s_2.cross(q_02),t_12);
    Eigen::Matrix4d S_03 = utility::matrix_exponential(s_3,-s_3.cross(q_03),t_23);
    Eigen::Matrix4d S_04 = utility::matrix_exponential(s_4,-s_4.cross(q_04),t_34);
    Eigen::Matrix4d S_05 = utility::matrix_exponential(s_5,-s_5.cross(q_05),t_45);
    Eigen::Matrix4d S_06 = utility::matrix_exponential(s_6,-s_6.cross(q_06),t_56);

    //Equation (4.13), page 140, MR pre-print 2019 - Space form of the product of exponentials formula
    Eigen::Matrix4d T = S_01*S_02*S_03*S_04*S_05*S_06*utility::transformation_matrix(R_sb,q_sb);
    return T;
}

Eigen::VectorXd ScrewsKinematicsSolver::ik_solve(const Eigen::Matrix4d &t_sd, const Eigen::VectorXd &j0)
{
    return ik_solve(t_sd, j0, [&](const std::vector<Eigen::VectorXd> &) { return 0u; });
}

//DONE: Implement ik_solve using screws.
//Algorithm without reference page 228-229, MR pre-print 2019
Eigen::VectorXd ScrewsKinematicsSolver::ik_solve(const Eigen::Matrix4d &t_sd, const Eigen::VectorXd &j0, const std::function<uint32_t(const std::vector<Eigen::VectorXd> &)> &solution_selector)
{
    std::pair<Eigen::VectorXd,double> pair;
    Eigen::Matrix4d V;
    Eigen::VectorXd theta = j0;
    Eigen::VectorXd twist(6);
    int i=1;
    double gamma = 1e-2;

    while ((twist.head(3).norm() > m_we or twist.tail(3).norm() > m_ve) and i<1e4) {
        V = fk_solve(theta).inverse()*t_sd;
        pair=utility::matrix_logarithm(V);
        twist = pair.first*pair.second;
        theta += gamma * body_jacobian(theta).completeOrthogonalDecomposition().pseudoInverse() * twist;
        i++;
    }

    for (int j=0; j<theta.size(); j++){
        while (theta(j)>=EIGEN_PI) {theta(j)-=2.0*EIGEN_PI;}
        while (theta(j)<=-EIGEN_PI) {theta(j)+=2.0*EIGEN_PI;}
    }

    return theta;
}

std::pair<Eigen::Matrix4d, std::vector<Eigen::VectorXd>> ScrewsKinematicsSolver::space_chain()
{
    return {m_m, m_screws};
}

//DONE: Implement body_chain() using space_chain().
std::pair<Eigen::Matrix4d, std::vector<Eigen::VectorXd>> ScrewsKinematicsSolver::body_chain()
{
    std::vector<Eigen::VectorXd> s_s = space_chain().second;
    //Equation without reference page 90, last equation in the page, MR pre-print 2019
    //T_de = T_ed.inverse()
    Eigen::Matrix4d M_b=space_chain().first.inverse();
    Eigen::MatrixXd A = utility::adjoint_matrix(M_b);
    std::vector<Eigen::VectorXd> s_b(s_s.size());
    //Equation without reference page 102, last equation in the page, MR pre-print 2019
    //S_a = [Ad_T_ab]*S_b
    for (int i=0; i<s_s.size(); i++) {s_b[i]= A*s_s[i];}
    return {M_b,s_b};
}

//DONE: Implement space_jacobian() using space_chain()
//Definition (5.1) page 177-178, MR pre-print 2019
//Equations (5.9) and (5.11) page 177-178, MR pre-print 2019
Eigen::MatrixXd ScrewsKinematicsSolver::space_jacobian(const Eigen::VectorXd &current_joint_positions)
{
    auto [M,s] = space_chain();
    Eigen::MatrixXd J(current_joint_positions.size(),current_joint_positions.size());
    Eigen::Matrix4d E = Eigen::Matrix4d::Identity();
    J.col(0)=s[0];
    for (int i=1; i<current_joint_positions.size(); i++) {
        E = E*utility::matrix_exponential(s[i-1].head(3),s[i-1].tail(3),current_joint_positions[i-1]);
        J.col(i) = utility::adjoint_matrix(E)*s[i];
    }
    return J;
}

//DONE: Implement body_jacobian() using body_chain()
//Definition (5.4) page 182-183, MR pre-print 2019
//Equations (5.16) and (5.18) page 182-183, MR pre-print 2019
Eigen::MatrixXd ScrewsKinematicsSolver::body_jacobian(const Eigen::VectorXd &current_joint_positions)
{
    auto [M,s] = body_chain();
    int n = current_joint_positions.size();
    Eigen::MatrixXd J(n,n);
    Eigen::Matrix4d E = Eigen::Matrix4d::Identity();
    J.col(n-1)=s[n-1];
    for (int i=n-2; i>=0; i--) {
        E = E*(utility::matrix_exponential(s[i+1].head(3),s[i+1].tail(3),current_joint_positions(i+1)).inverse());
        J.col(i) = utility::adjoint_matrix(E)*s[i];
    }
    return J;
}

