#ifndef AIS4104_URDF_LOADERS_H
#define AIS4104_URDF_LOADERS_H

#include "app/examples/tracikkinematicssolver.h"

#include "app/implementations/screwskinematicssolver.h"

#include <trac_ik/trac_ik.hpp>

#include <threepp/objects/Robot.hpp>

#include <threepp/loaders/URDFLoader.hpp>
#include <threepp/loaders/AssimpLoader.hpp>

#include <filesystem>

namespace AIS4104 {

inline std::shared_ptr<threepp::Robot> load_threepp_robot(const std::filesystem::path &urdf_path, bool show_mesh = false)
{
    if(!exists(urdf_path))
        throw std::runtime_error("URDF file not found: " + urdf_path.string());

    threepp::URDFLoader loader;
    threepp::AssimpLoader assimpLoader;
    auto robot = loader.load(assimpLoader, urdf_path);
    robot->rotation.x = -threepp::math::PI / 2.f;
    robot->showColliders(show_mesh);

    return robot;
}

//TASK: Write a function or class that constructs the screws and zero configuration pose matrix M from the specified URDF path
inline std::shared_ptr<ScrewsKinematicsSolver> load_robot_chain_screws(const std::filesystem::path &urdf_path)
{
    return {};
}

//TASK: Write a function or class that creates the KDL kinematic chain from the specified URDF path
inline std::shared_ptr<TracIkKinematicsSolver> load_robot_chain_kdl(const std::filesystem::path &urdf_path, bool show_mesh = false)
{
    /*
    if(!exists(urdf_path))
        throw std::runtime_error("URDF file not found: " + urdf_path.string());

    threepp::URDFLoader loader;
    threepp::AssimpLoader assimpLoader;
    auto robot = loader.load(assimpLoader, urdf_path);
    robot->rotation.x = -threepp::math::PI / 2.f;
    robot->showColliders(show_mesh);

    auto jointObject = std::make_shared<threepp::Object3D>();
    std::vector<threepp::JointInfo> joint = robot->getArticulatedJointInfo();
    robot->geometry()

    double Z_1 = 0.400; double X_1 = 0.025;
    double X_2 = 0.455;
    double Z_3 = 0.035;
    double X_4 = 0.420;
    double X_5 = 0.08;

    auto c = std::make_shared<KDL::Chain>();
    c->addSegment(KDL::Segment(KDL::Joint(KDL::Joint::JointType::Fixed), KDL::Frame(KDL::Vector(0.0, 0.0, 0.0))));
    c->addSegment(KDL::Segment(KDL::Joint(KDL::Joint::JointType::RotZ), KDL::Frame(KDL::Vector(X_1, 0.0, Z_1))));
    c->addSegment(KDL::Segment(KDL::Joint(KDL::Joint::JointType::RotY), KDL::Frame(KDL::Vector(X_2, 0.0, 0.0))));
    c->addSegment(KDL::Segment(KDL::Joint(KDL::Joint::JointType::RotY), KDL::Frame(KDL::Vector(0.0, 0.0, Z_3))));
    c->addSegment(KDL::Segment(KDL::Joint(KDL::Joint::JointType::RotX), KDL::Frame(KDL::Vector(X_4, 0.0, 0.0))));
    c->addSegment(KDL::Segment(KDL::Joint(KDL::Joint::JointType::RotY), KDL::Frame(KDL::Vector(X_5, 0.0, 0.0))));
    c->addSegment(KDL::Segment(KDL::Joint(KDL::Joint::JointType::RotX), KDL::Frame(KDL::Vector(0.0, 0.0, 0.0))));

    Simulation::JointLimits limits
    {
        utility::to_eigen_vectord(std::vector<double>{180.0, 180.0, 180.0, 360.0, 360.0, 360.0}) * utility::deg_to_rad,
        utility::to_eigen_vectord(std::vector<double>{90.0, 90.0, 90.0, 180.0, 180.0, 180.0}) * utility::deg_to_rad,
        utility::to_eigen_vectord(std::vector<double>{robot->getJointRange(0,false).first,
            robot->getJointRange(1,false).first, robot->getJointRange(2,false).first,
            robot->getJointRange(3,false).first, robot->getJointRange(4,false).first,
            robot->getJointRange(5,false).first}) * utility::deg_to_rad,
        utility::to_eigen_vectord(std::vector<double>{robot->getJointRange(0,false).second,
            robot->getJointRange(1,false).second, robot->getJointRange(2,false).second,
            robot->getJointRange(3,false).second, robot->getJointRange(4,false).second,
            robot->getJointRange(5,false).second}) * utility::deg_to_rad
    };

    return std::make_shared<TracIkKinematicsSolver>(c, limits);
    */
    return {};
}

}

#endif
