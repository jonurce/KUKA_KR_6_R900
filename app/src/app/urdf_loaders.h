#ifndef AIS4104_URDF_LOADERS_H
#define AIS4104_URDF_LOADERS_H

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
inline std::pair<Eigen::Matrix4d, std::vector<Eigen::VectorXd>> load_robot_chain_screws(const std::filesystem::path &urdf_path)
{
    return {};
}

//TASK: Write a function or class that creates the KDL kinematic chain from the specified URDF path
inline std::shared_ptr<KDL::Chain> load_robot_chain_kdl(const std::filesystem::path &urdf_path)
{
    return nullptr;
}

}

#endif
