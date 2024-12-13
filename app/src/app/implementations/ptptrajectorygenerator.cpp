#include "ptptrajectorygenerator.h"

using namespace AIS4104;

void PTPTrajectoryGenerator::stop()
{
    m_stopped = true;
}

bool PTPTrajectoryGenerator::has_reached_endpoint() const
{
    return m_current == m_total || m_stopped;
}

//Formulas without reference, maximum velocity and acceleration, page 332, MR pre-print 2019
bool PTPTrajectoryGenerator::plan_trajectory(const Simulation::JointLimits &limits, double velocity_factor)
{
    if(m_w0.isApprox(m_w1))
    {
        m_total = std::chrono::nanoseconds(0);
        m_stopped = true;
    }
    else
    {
        m_stopped = false;
        Eigen::VectorXd velocity_limits = limits.velocity * velocity_factor;
        Eigen::VectorXd acceleration_limits = limits.acceleration * velocity_factor;
        Eigen::VectorXd diff = (m_w1 - m_w0).cwiseAbs();
        //... equations from the book implemented here
        Eigen::VectorXd times_vel = 3.0/2.0*(diff.array() / velocity_limits.array());
        Eigen::VectorXd times_accel = sqrt(6.0*(diff.array() / velocity_limits.array()));
        Eigen::VectorXd times(times_vel.size()+times_accel.size());
        times<<times_vel,times_accel;
        uint64_t duration = times.maxCoeff()*1000000000.0;
        m_total = std::chrono::nanoseconds(duration);
    }
    return true;
}

//Equation (9.11) page 332, MR pre-print 2019
Eigen::VectorXd PTPTrajectoryGenerator::joint_positions(std::chrono::nanoseconds delta_t)
{
    //... equation from the book implemented here
    if(m_stopped)
    {
        double s = 3.0*pow(static_cast<double>(m_current.count()),2.0) / pow(static_cast<double>(m_total.count()),2.0);
        s -=2.0*pow(static_cast<double>(m_current.count()),3.0) / (pow(static_cast<double>(m_total.count()),3.0));
        s = std::clamp(s, 0.0, 1.0);
        return m_w0 + s * (m_w1 - m_w0);
    }
    m_current = std::clamp(m_current + delta_t, std::chrono::nanoseconds(0), m_total);
    double s = 3.0*pow(static_cast<double>(m_current.count()),2.0) / pow(static_cast<double>(m_total.count()),2.0);
    s -=2.0*pow(static_cast<double>(m_current.count()),3.0) / (pow(static_cast<double>(m_total.count()),3.0));
    s = std::clamp(s, 0.0, 1.0);
    return m_w0 + s * (m_w1 - m_w0);
}
