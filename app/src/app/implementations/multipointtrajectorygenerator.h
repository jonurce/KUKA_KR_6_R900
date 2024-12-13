#ifndef MULTIPOINTTRAJECTORYGENERATOR_H
#define MULTIPOINTTRAJECTORYGENERATOR_H

#include <simulation/trajectorygenerator.h>

namespace AIS4104 {

    class MPTrajectoryGenerator : public Simulation::MultipointTrajectoryGenerator
    {
    public:
        using MultipointTrajectoryGenerator::MultipointTrajectoryGenerator;

        void stop() override;
        bool has_reached_endpoint() const override;

        bool plan_trajectory(const Simulation::JointLimits &limits, double velocity_factor) override;

        Eigen::VectorXd joint_positions(std::chrono::nanoseconds delta_t) override;

    private:
        std::atomic<bool> m_stopped;
        std::chrono::nanoseconds m_total;
        std::chrono::nanoseconds m_current;
        std::vector<uint64_t> m_times;
        int i_segment;
        double DELTA_T;
        Eigen::VectorXd a_j0;
        Eigen::VectorXd a_j1;
        Eigen::VectorXd a_jplus;
        Eigen::VectorXd a_j2;
        Eigen::VectorXd a_j3;
        double acumulated_delta_t;
    };

}

#endif
