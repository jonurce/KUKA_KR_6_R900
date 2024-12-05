#include "utility/math.h"
#include "utility/vectors.h"

namespace AIS4104::utility {

//TASK: Implement the following function definitions

bool floatEquals(double a, double b)
{
    return std::abs(a - b) < equal_precision;
}

Eigen::Vector3d euler_zyx_from_rotation_matrix(const Eigen::Matrix3d &r)
{
    double a = 0.0; //Z axis Alpha
    double b = 0.0; //Y axis Beta
    double c = 0.0; //X axis Gamma

    if(floatEquals(r(2,0),1.0))
    {
        b = -EIGEN_PI / 2.0;
        c = -std::atan2(r(0,1), r(1,1));
    }
    else if(floatEquals(r(2,0),-1.0))
    {
        b = EIGEN_PI / 2.0;
        c = std::atan2(r(0,1), r(1,1));
    }
    else
    {
        b = std::atan2(-r(2,0), std::sqrt(r(0,0)*r(0,0)+r(1,0)*r(1,0)));
        a = std::atan2(r(1,0), r(0,0));
        c = std::atan2(r(2,1), r(2,2));
    }

    return Eigen::Vector3d{a, b, c};
}

Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d &v)
{
    Eigen::Matrix3d matrix;
    matrix << 0, -v(2), v(1),
    v(2), 0, -v(0),
    -v(1), v(0), 0;
    return matrix;
}

Eigen::Vector3d from_skew_symmetric(const Eigen::Matrix3d &m)
{
    Eigen::Vector3d vector{m(2,1), m(0,2), m(1,0)};
    return vector;
}

Eigen::MatrixXd adjoint_matrix(const Eigen::Matrix3d &r, const Eigen::Vector3d &p)
{
    Eigen::MatrixXd ad(6,6);
    ad.block<3,3>(0,0) = r;
    ad.block<3,3>(0,3) = Eigen::Matrix3d::Zero();
    ad.block<3,3>(3,3) = r;
    ad.block<3,3>(3,0) = skew_symmetric(p) * r;
    return ad;
}

Eigen::MatrixXd adjoint_matrix(const Eigen::Matrix4d &tf)
{
    Eigen::Matrix3d R = tf.block<3,3>(0,0);
    Eigen::MatrixXd ad(6,6);
    ad.block<3,3>(0,0) = R;
    ad.block<3,3>(0,3) = Eigen::Matrix3d::Zero();
    ad.block<3,3>(3,3) = R;
    ad.block<3,3>(3,0) = skew_symmetric(tf.block<3,1>(0,3)) * R;
    return ad;
}

Eigen::VectorXd adjoint_map(const Eigen::VectorXd &twist, const Eigen::Matrix4d &tf)
{
    return Eigen::VectorXd::Zero(6);
}

Eigen::VectorXd twist(const Eigen::Vector3d &w, const Eigen::Vector3d &v)
{
    Eigen::VectorXd tw(6);
    tw << w,v;
    return tw;
}

Eigen::VectorXd twist(const Eigen::Vector3d &q, const Eigen::Vector3d &s, double h, double angular_velocity)
{
    Eigen::VectorXd tw(6);
    tw << angular_velocity*s,angular_velocity*(-s.cross(q) + h * s);
    return tw;
}

Eigen::Matrix4d twist_matrix(const Eigen::Vector3d &w, const Eigen::Vector3d &v)
{
    return Eigen::Matrix4d::Identity();
}

Eigen::Matrix4d twist_matrix(const Eigen::VectorXd &twist)
{
    return Eigen::Matrix4d::Identity();
}

Eigen::VectorXd screw_axis(const Eigen::Vector3d &w, const Eigen::Vector3d &v)
{
    return Eigen::VectorXd::Zero(6);
}

Eigen::VectorXd screw_axis(const Eigen::Vector3d &q, const Eigen::Vector3d &s, double h)
{
    return Eigen::VectorXd::Zero(6);
}

Eigen::Matrix3d matrix_exponential(const Eigen::Vector3d &w, double theta)
{
    return Eigen::Matrix3d::Identity();
}

Eigen::Matrix4d matrix_exponential(const Eigen::Vector3d &w, const Eigen::Vector3d &v, double theta)
{
    return Eigen::Matrix4d::Identity();
}

Eigen::Matrix4d matrix_exponential(const Eigen::VectorXd &screw, double theta)
{
    return Eigen::Matrix4d::Identity();
}

std::pair<Eigen::Vector3d, double> matrix_logarithm(const Eigen::Matrix3d &r)
{
    return std::make_pair(Eigen::Vector3d::Zero(), 0);
}

std::pair<Eigen::VectorXd, double> matrix_logarithm(const Eigen::Matrix3d &r, const Eigen::Vector3d &p)
{
    return std::make_pair(Eigen::VectorXd::Zero(6), 0);
}

std::pair<Eigen::VectorXd, double> matrix_logarithm(const Eigen::Matrix4d &tf)
{
    return std::make_pair(Eigen::VectorXd::Zero(6), 0);
}

Eigen::Matrix3d rotate_x(double radians)
{
    return Eigen::Matrix3d::Identity();
}

Eigen::Matrix3d rotate_y(double radians)
{
    return Eigen::Matrix3d::Identity();
}

Eigen::Matrix3d rotate_z(double radians)
{
    return Eigen::Matrix3d::Identity();
}

Eigen::Matrix3d rotation_matrix_from_frame_axes(const Eigen::Vector3d &x, const Eigen::Vector3d &y, const Eigen::Vector3d &z)
{
    return Eigen::Matrix3d::Identity();
}

Eigen::Matrix3d rotation_matrix_from_euler_zyx(const Eigen::Vector3d &e)
{
    return Eigen::Matrix3d::Identity();
}

Eigen::Matrix3d rotation_matrix_from_axis_angle(const Eigen::Vector3d &axis, double radians)
{
    return Eigen::Matrix3d::Identity();
}

Eigen::Matrix3d rotation_matrix(const Eigen::Matrix4d &tf)
{
    return Eigen::Matrix3d::Identity();
}

Eigen::Matrix4d transformation_matrix(const Eigen::Vector3d &p)
{
    return Eigen::Matrix4d::Identity();
}

Eigen::Matrix4d transformation_matrix(const Eigen::Matrix3d &r)
{
    return Eigen::Matrix4d::Identity();
}

Eigen::Matrix4d transformation_matrix(const Eigen::Matrix3d &r, const Eigen::Vector3d &p)
{
    return Eigen::Matrix4d::Identity();
}

}
