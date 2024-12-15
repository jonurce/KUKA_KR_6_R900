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

//Formulas without reference for Third order polynomial, maximum velocity and acceleration, page 332, MR pre-print 2019
//Own made formulas for Fifth order polynomial
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
        //Third order polynomial:
        /*
        Eigen::VectorXd times_vel = 3.0/2.0*(diff.array() / velocity_limits.array());
        Eigen::VectorXd times_accel = sqrt(6.0*(diff.array() / velocity_limits.array()));
        */
        //Fifth order polynomial:
        Eigen::VectorXd times_vel = 15.0/8.0*(diff.array() / velocity_limits.array());
        Eigen::VectorXd times_accel = sqrt(10.0/sqrt(3.0)*(diff.array() / velocity_limits.array()));

        Eigen::VectorXd times(times_vel.size()+times_accel.size());
        times<<times_vel,times_accel;
        uint64_t duration = times.maxCoeff()*1000000000.0;
        m_total = std::chrono::nanoseconds(duration);
    }
    return true;
}

//Equation (9.11) page 332, MR pre-print 2019 - Third order polynomial
//Own made formulas for Fifth order polynomial
Eigen::VectorXd PTPTrajectoryGenerator::joint_positions(std::chrono::nanoseconds delta_t)
{
    //... equation from the book implemented here
    if(m_stopped)
    {
        //Third order polynomial:
        /*
        double s = 3.0*pow(static_cast<double>(m_current.count()),2.0) / pow(static_cast<double>(m_total.count()),2.0);
        s -=2.0*pow(static_cast<double>(m_current.count()),3.0) / (pow(static_cast<double>(m_total.count()),3.0));
        */
        //Fifth order polynomial:
        double a_0 = 0.0;
        double a_1 = 0.0;
        double a_2 = 0.0;
        double a_3 = 10.0/pow(static_cast<double>(m_total.count()),3.0);
        double a_4 = -15.0/pow(static_cast<double>(m_total.count()),4.0);
        double a_5 = 6.0/pow(static_cast<double>(m_total.count()),5.0);
        double s = a_0+a_1*static_cast<double>(m_current.count())+a_2*pow(static_cast<double>(m_current.count()),2.0)+
                    a_3*pow(static_cast<double>(m_current.count()),3.0)+a_4*pow(static_cast<double>(m_current.count()),4.0)+
                    a_5*pow(static_cast<double>(m_current.count()),5.0);

        s = std::clamp(s, 0.0, 1.0);
        return m_w0 + s * (m_w1 - m_w0);
    }
    m_current = std::clamp(m_current + delta_t, std::chrono::nanoseconds(0), m_total);

    //Third order polynomial:
    /*
    double s = 3.0*pow(static_cast<double>(m_current.count()),2.0) / pow(static_cast<double>(m_total.count()),2.0);
    s -=2.0*pow(static_cast<double>(m_current.count()),3.0) / (pow(static_cast<double>(m_total.count()),3.0));
    */
    //Fifth order polynomial:
    double a_0 = 0.0;
    double a_1 = 0.0;
    double a_2 = 0.0;
    double a_3 = 10.0/pow(static_cast<double>(m_total.count()),3.0);
    double a_4 = -15.0/pow(static_cast<double>(m_total.count()),4.0);
    double a_5 = 6.0/pow(static_cast<double>(m_total.count()),5.0);
    double s = a_0+a_1*static_cast<double>(m_current.count())+a_2*pow(static_cast<double>(m_current.count()),2.0)+
                a_3*pow(static_cast<double>(m_current.count()),3.0)+a_4*pow(static_cast<double>(m_current.count()),4.0)+
                a_5*pow(static_cast<double>(m_current.count()),5.0);

    s = std::clamp(s, 0.0, 1.0);
    return m_w0 + s * (m_w1 - m_w0);
}
