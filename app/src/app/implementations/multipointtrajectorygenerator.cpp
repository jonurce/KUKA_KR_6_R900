#include "multipointtrajectorygenerator.h"

#include "utility/csv.h"
#include "utility/math.h"

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
    //Calculate times for each waypoint, based on maximum velocities and acceleration of joints
    Eigen::VectorXd velocity_limits = limits.velocity * velocity_factor;
    Eigen::VectorXd acceleration_limits = limits.acceleration * velocity_factor;
    Eigen::VectorXd diff (m_waypoints[0].size());
    Eigen::VectorXd times_vel(m_waypoints[0].size());
    Eigen::VectorXd times_accel(m_waypoints[0].size());
    Eigen::VectorXd times(times_vel.size()+times_accel.size());
    m_times = {static_cast<uint64_t>(0)};
    for (int i = 0; i < (m_waypoints.size()-1); i++){
        if (m_waypoints[i].isApprox(m_waypoints[i+1]))
        {
            m_total = std::chrono::nanoseconds(0);
            m_stopped = true;
            m_times.push_back(m_times[i]);
        }
        else {
            m_stopped = false;
            //... equations from the book implemented here
            diff = (m_waypoints[i+1] - m_waypoints[i]).cwiseAbs();
            times_vel = 3.0/2.0*(diff.array() / velocity_limits.array());
            times_accel = sqrt(6.0*(diff.array() / velocity_limits.array()));
            times<<times_vel,times_accel;
            m_times.push_back(m_times[i]+static_cast<uint64_t>(times.maxCoeff()*1000000000.0));
            m_total = std::chrono::nanoseconds(m_times.back());
        }
    }
    //Check if the waypoints in the trajectory are within the limits
    Eigen::VectorXd upper_limits = limits.upper_position;
    Eigen::VectorXd lower_limits = limits.lower_position;
    for (int i = 0; i < m_waypoints.size(); i++) {
        for (int j = 0; j < upper_limits.size(); j++) {
            if (m_waypoints[i][j] >= upper_limits[j] || m_waypoints[i][j] <= lower_limits[j]) {
                m_stopped = true;
                break;
            }
        }
    }

    return true;
}

//Equation (9.25), (9.26), (9.27), (9.28) and (9.29) page 337, MR pre-print 2019
Eigen::VectorXd MPTrajectoryGenerator::joint_positions(std::chrono::nanoseconds delta_t) {
    //... equation from the book implemented here
    j = std::clamp(j, 0, static_cast<int>(m_times.size()));
    DT = std::clamp(DT, 0.0, abs(DT));
    sum_DT = std::clamp(sum_DT, 0.0, abs(sum_DT));

    if(!m_stopped) {
        delta_t = std::clamp(delta_t, std::chrono::nanoseconds(0),std::chrono::nanoseconds(200000000));
        m_current = std::clamp(m_current, -delta_t, m_total);
        m_current = std::clamp(m_current + delta_t, std::chrono::nanoseconds(0), m_total);
        double dt = static_cast<double>(m_current.count())/1000000000.0-sum_DT;
        dt = std::clamp(dt, 0.0, DT);

        if (j < m_times.size()-1) {
            if (static_cast<double>(m_current.count()) > static_cast<double>(m_times[j])) {
                sum_DT += DT;
                DT = static_cast<double>(m_times[j+1]-m_times[j])/1000000000.0;
                a_j0 = m_waypoints[j];
                if (j==0) {
                    a_j1 = Eigen::VectorXd::Zero(m_waypoints[0].size());
                }
                else {
                    a_j1 = a_jplus;
                }
                if (j+1 == m_times.size()-1) {
                    a_jplus = Eigen::VectorXd::Zero(m_waypoints[0].size());
                }
                else {
                    a_jplus = (m_waypoints[j+2]-m_waypoints[j])/
                        (static_cast<double>(m_times[j+2]-m_times[j])/1000000000.0);
                }
                a_j2 = (3*m_waypoints[j+1]-3*a_j0-2*a_j1*DT-a_jplus*DT)/pow(DT,2);
                a_j3 = (2*a_j0+(a_j1+a_jplus)*DT-2*m_waypoints[j+1])/pow(DT,3);
                j+=1;
            }
        }
        return a_j0+a_j1*dt+a_j2*pow(dt,2.0)+a_j3*pow(dt,3.0);
    }
}
