#ifndef PTPTRAJECTORYGENERATOR_H
#define PTPTRAJECTORYGENERATOR_H

#include <simulation/trajectorygenerator.h>

namespace AIS4104 {

    class PTPTrajectoryGenerator : public Simulation::PointToPointTrajectoryGenerator
    {
    public:
        using PointToPointTrajectoryGenerator::PointToPointTrajectoryGenerator;

        void stop() override;
        bool has_reached_endpoint() const override;

        bool plan_trajectory(const Simulation::JointLimits &limits, double velocity_factor) override;

        Eigen::VectorXd joint_positions(std::chrono::nanoseconds delta_t) override;

    private:
        std::atomic<bool> m_stopped;
        std::chrono::nanoseconds m_total;
        std::chrono::nanoseconds m_current;
    };

}

#endif
