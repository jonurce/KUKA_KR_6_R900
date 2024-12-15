#include "app/implementations/motionplannerimpl.h"

#include <iostream>

#include "app/examples/ptplinetrajectorygeneratorexample.h"
#include "app/implementations/ptptrajectorygenerator.h"
#include "app/implementations/multipointtrajectorygenerator.h"

#include <utility/vectors.h>

#include <spdlog/spdlog.h>

#include <iostream>

#include "utility/math.h"

using namespace AIS4104;

//TASK: Remove the std::cout prints as the functions have been implemented. (e.g., std::cout << "MotionPlannerImpl::FUNCTION_NAME" << std::endl;)

MotionPlannerImpl::MotionPlannerImpl(const Simulation::Robot &robot)
: m_robot(robot)
{
}

Eigen::VectorXd MotionPlannerImpl::task_space_pose(const Eigen::Matrix4d &pose)
{
    return m_robot.ik_solve_pose(pose, m_robot.joint_positions());
}

//DONE: Implement a function that calculates the pose of the given screw, and solves the IK to obtain the joint positions
Eigen::VectorXd MotionPlannerImpl::task_space_screw(const Eigen::Vector3d &w, const Eigen::Vector3d &q, double theta, double h)
{
    //std::cout << "MotionPlannerImpl::task_space_screw:" << std::endl << w.transpose() << std::endl << q.transpose() << std::endl << theta << std::endl << h << std::endl << std::endl;
    Eigen::VectorXd screw = utility::screw_axis(q,w.normalized(),h);
    Eigen::Matrix4d T_w_desired = utility::matrix_exponential(screw.block<3,1>(0,0),screw.block<3,1>(3,0),theta);
    return m_robot.ik_solve_pose(T_w_desired,m_robot.joint_positions());
}

//DONE: Implement jogging in tool frame from the start pose along the displacement and rotation
Eigen::VectorXd MotionPlannerImpl::tool_frame_displace(const Eigen::Matrix4d &tw_pose, const Eigen::Vector3d &tf_offset, const Eigen::Vector3d &tf_zyx)
{
    //std::cout << "MotionPlannerImpl::tool_frame_displace:" << std::endl << tw_pose << std::endl << tf_offset.transpose() << std::endl << tf_zyx.transpose() << std::endl << std::endl;
    Eigen::Matrix4d T_w_desired = tw_pose*utility::transformation_matrix(utility::rotation_matrix_from_euler_zyx(tf_zyx),tf_offset);
    return m_robot.ik_solve_pose(T_w_desired,m_robot.joint_positions());
}

//DONE: Implement a P2P trajectory generator from the current configuration to the target.
std::shared_ptr<Simulation::TrajectoryGenerator> MotionPlannerImpl::task_space_ptp_trajectory(const Eigen::Vector3d &pos, const Eigen::Vector3d &euler_zyx)
{
    Eigen::Matrix4d T_w_desired = utility::transformation_matrix(utility::rotation_matrix_from_euler_zyx(euler_zyx),pos);
    Eigen::VectorXd joint_desired = m_robot.ik_solve_pose(T_w_desired, m_robot.joint_positions());
    return std::make_shared<PTPTrajectoryGenerator>(m_robot.joint_positions(),joint_desired);
}

//TASK: Implement a LIN trajectory generator from the current configuration to the target.
std::shared_ptr<Simulation::TrajectoryGenerator> MotionPlannerImpl::task_space_lin_trajectory(const Eigen::Vector3d &pos, const Eigen::Vector3d &euler_zyx)
{
    Eigen::Matrix4d T_w_desired;
    Eigen::VectorXd current_joints = m_robot.joint_positions();
    std::vector<Eigen::VectorXd> waypoints = {Eigen::VectorXd::Zero(current_joints.size())};
    waypoints={m_robot.joint_positions()};
    Eigen::Vector3d current_pos = m_robot.current_position();
    Eigen::Vector3d current_zyx = m_robot.current_orientation_zyx();
    double n_segments = 2.0;
    for (int i = 0; i < n_segments; i++) {
        current_pos += (pos-current_pos)/(n_segments-i);
        current_zyx += (euler_zyx-current_zyx)/(n_segments-i);
        T_w_desired = utility::transformation_matrix(utility::rotation_matrix_from_euler_zyx(current_zyx), current_pos);
        current_joints = m_robot.ik_solve_pose(T_w_desired, current_joints);
        waypoints.push_back(current_joints);
    }
    return std::make_shared<MPTrajectoryGenerator>(waypoints);
}

//TASK: Implement a screw trajectory generator from the current configuration to the target.
std::shared_ptr<Simulation::TrajectoryGenerator> MotionPlannerImpl::task_space_screw_trajectory(const Eigen::Vector3d &w, const Eigen::Vector3d &q, double theta, double h)
{
    Eigen::Matrix4d T_w_desired;
    Eigen::VectorXd current_joints = m_robot.joint_positions();
    std::vector<Eigen::VectorXd> waypoints = {Eigen::VectorXd::Zero(current_joints.size())};
    waypoints={m_robot.joint_positions()};
    double current_theta = 0.0;
    double n_segments = 2.0;
    for (int i = 0; i < n_segments; i++) {
        current_theta += (theta-current_theta)/(n_segments-i);
        T_w_desired = utility::matrix_exponential(utility::screw_axis(q,w.normalized(),h),current_theta);
        current_joints = m_robot.ik_solve_pose(T_w_desired, current_joints);
        waypoints.push_back(current_joints);
    }
    return std::make_shared<MPTrajectoryGenerator>(waypoints);
}

std::shared_ptr<Simulation::TrajectoryGenerator> MotionPlannerImpl::task_space_trajectory(const std::vector<Eigen::Matrix4d> &waypoints)
{
    std::vector<Eigen::VectorXd> joint_positions;
    auto idx = 0u;
    if(waypoints.empty())
    {
        spdlog::error("[MotionPlannerImpl::task_space_trajectory] No waypoints given.");
        return nullptr;
    }
    if(waypoints.size() == 1u)
        joint_positions.push_back(m_robot.joint_positions());
    else
        joint_positions.push_back(m_robot.ik_solve_pose(waypoints[idx++], m_robot.joint_positions()));

    for(auto i = idx; i < waypoints.size(); i++)
        joint_positions.push_back(m_robot.ik_solve_pose(waypoints.at(i), joint_positions.at(i)));
    return joint_space_trajectory(joint_positions);
}

//TASK: Implement joint space trajectory generators.
// a) Implement a point to point generator which does not have infinite acceleration (e.g., polynomial time scaling).
// b) Change the use of the standard PTPLineTrajectoryGeneratorExample to your custom trajectory generator.
// c) Implement a multipoint trajectory generator for trajectories with multiple waypoints.
std::shared_ptr<Simulation::TrajectoryGenerator> MotionPlannerImpl::joint_space_trajectory(const std::vector<Eigen::VectorXd> &waypoints)
{
    std::shared_ptr<Simulation::TrajectoryGenerator> trajectory_generator;
    if(waypoints.size() == 2u)
        trajectory_generator = std::make_shared<PTPLineTrajectoryGeneratorExample>(waypoints.front(), waypoints.back());
    else if(waypoints.size() == 1u)
        trajectory_generator = std::make_shared<PTPLineTrajectoryGeneratorExample>(m_robot.joint_positions(), waypoints.front());
    else if(waypoints.empty())
    {
        spdlog::error("[MotionPlannerImpl::joint_space_trajectory] No waypoints given.");
    }
    else
    {
        spdlog::error("[MotionPlannerImpl::joint_space_trajectory] Multipoint trajectories are not implemented.");
    }
    return trajectory_generator;
}
