#include "multipointtrajectorygenerator.h"

using namespace AIS4104;

void MPTrajectoryGenerator::stop()
{
    m_stopped = true;
}

bool MPTrajectoryGenerator::has_reached_endpoint() const
{
    return m_current == m_total || m_stopped;
}

//Formulas without reference, maximum velocity and acceleration, page 332, MR pre-print 2019
bool MPTrajectoryGenerator::plan_trajectory(const Simulation::JointLimits &limits, double velocity_factor)
{
    Eigen::VectorXd velocity_limits = limits.velocity * velocity_factor;
    Eigen::VectorXd acceleration_limits = limits.acceleration * velocity_factor;
    Eigen::VectorXd diff (m_waypoints[0].size());
    Eigen::VectorXd times_vel(m_waypoints[0].size());
    Eigen::VectorXd times_accel(m_waypoints[0].size());
    Eigen::VectorXd times(times_vel.size()+times_accel.size());
    for (int i = 0; i < (m_waypoints.size()-1); i++){
        if (m_waypoints[i].isApprox(m_waypoints[i+1]))
        {
            m_total = std::chrono::nanoseconds(0);
            m_stopped = true;
            break;
        }
        else {
            m_stopped = false;
            //... equations from the book implemented here
            diff = (m_waypoints[i+1] - m_waypoints[i]).cwiseAbs();
            times_vel = 3.0/2.0*(diff.array() / velocity_limits.array());
            times_accel = sqrt(6.0*(diff.array() / velocity_limits.array()));
            times<<times_vel,times_accel;
            if (i==0) {
                m_times = {static_cast<uint64_t>(times.maxCoeff()*1000000000.0)};
            }
            else {
                m_times.push_back(m_times[i-1]+static_cast<uint64_t>(times.maxCoeff()*1000000000.0));
            }
        }
        if (i == m_waypoints.size()-2) {
            break;
        }
    }
    m_total = std::chrono::nanoseconds(m_times.back());
    return true;
}

//Equation (9.25), (9.26), (9.27), (9.28) and (9.29) page 337, MR pre-print 2019
Eigen::VectorXd MPTrajectoryGenerator::joint_positions(std::chrono::nanoseconds delta_t)
{
    //... equation from the book implemented here
    i_segment = std::clamp(i_segment, 0, static_cast<int>(m_times.size()));
    if(i_segment == 0) {
        DELTA_T = static_cast<double>(m_times[i_segment]);
        a_j0 = m_waypoints[i_segment];
        a_j1 = Eigen::VectorXd::Zero(m_waypoints[0].size());
        if(m_waypoints.size() <= 2) {
            a_jplus = Eigen::VectorXd::Zero(m_waypoints[0].size());
        }
        else {
            a_jplus = m_waypoints[i_segment+1]/DELTA_T;
        }
        a_j2 = (3*m_waypoints[i_segment+1]-3*a_j0-2*a_j1*DELTA_T-a_jplus*DELTA_T)/pow(DELTA_T,2);
        a_j3 = (2*a_j0+(a_j1+a_jplus)*DELTA_T-2*m_waypoints[i_segment+1])/pow(DELTA_T,3);
        i_segment += 1;
        acumulated_delta_t = 0.0;
    }

    if (i_segment < m_times.size()) {
        //m_times[i_segment] < static_cast<uint64_t>(m_total.count()) &&
        if(static_cast<uint64_t>(m_current.count()) == m_times[i_segment]) {
            DELTA_T = static_cast<double>(m_times[i_segment]-m_times[i_segment-1]);
            a_j0 = m_waypoints[i_segment];
            a_j1 = a_jplus;
            if (i_segment == m_times.size()-1) {
                a_jplus = Eigen::VectorXd::Zero(m_waypoints[0].size());
            }
            else {
                a_jplus = (m_waypoints[i_segment+1]-m_waypoints[i_segment-1])/
                    static_cast<double>(m_times[i_segment]-m_times[i_segment-1]);
            }
            a_j2 = (3*m_waypoints[i_segment+1]-3*a_j0-2*a_j1*DELTA_T-a_jplus*DELTA_T)/pow(DELTA_T,2);
            a_j3 = (2*a_j0+(a_j1+a_jplus)*DELTA_T-2*m_waypoints[i_segment+1])/pow(DELTA_T,3);
            acumulated_delta_t = 0.0;
            i_segment += 1;
        }
    }

    m_current = std::clamp(m_current + delta_t, std::chrono::nanoseconds(0), m_total);
    acumulated_delta_t += static_cast<double>(delta_t.count());
    if(m_stopped)
    {
        return a_j0+a_j1*acumulated_delta_t+a_j2*pow(acumulated_delta_t,2.0)+a_j3*pow(acumulated_delta_t,3.0);
    }
    return a_j0+a_j1*acumulated_delta_t+a_j2*pow(acumulated_delta_t,2.0)+a_j3*pow(acumulated_delta_t,3.0);
}
