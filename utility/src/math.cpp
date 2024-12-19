#include "utility/math.h"
#include "utility/vectors.h"

namespace AIS4104::utility {

//DONE: Implement the following function definitions
bool floatEquals(double a, double b)
{
    return std::abs(a - b) < equal_precision;
}

double cot(double rad)
{
    double tan = std::tan(rad);
    if(!floatEquals(tan,0.0)) {return 1.0/tan;}
    else {throw std::runtime_error("Infinite cotangent detected");}
}

//Algorithm in section B.1.1 page 579, MR pre-print 2019
Eigen::Vector3d euler_zyx_from_rotation_matrix(const Eigen::Matrix3d &r)
{
    //... equations from the book implemented here
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

//Definition (3.7) page 75, MR 3rd print 2019
Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d &v)
{
    //... equation from the book implemented here
    Eigen::Matrix3d matrix;
    matrix << 0, -v(2), v(1),
    v(2), 0, -v(0),
    -v(1), v(0), 0;
    return matrix;
}

//Definition (3.7) page 75, MR 3rd print 2019
Eigen::Vector3d from_skew_symmetric(const Eigen::Matrix3d &m)
{
    //... equation from the book implemented here
    Eigen::Vector3d vector{m(2,1), m(0,2), m(1,0)};
    return vector;
}

//Definition (3.20) page 98, MR pre-print 2019 - Adjoint matrix
Eigen::MatrixXd adjoint_matrix(const Eigen::Matrix3d &r, const Eigen::Vector3d &p)
{
    //... equation from the book implemented here
    Eigen::MatrixXd ad(6,6);
    ad.block<3,3>(0,0) = r;
    ad.block<3,3>(0,3) = Eigen::Matrix3d::Zero();
    ad.block<3,3>(3,3) = r;
    ad.block<3,3>(3,0) = skew_symmetric(p) * r;
    return ad;
}

//Definition (3.20) page 98, MR pre-print 2019 - Adjoint matrix
Eigen::MatrixXd adjoint_matrix(const Eigen::Matrix4d &tf)
{
    //... equations from the book implemented here
    return adjoint_matrix(tf.block<3,3>(0,0),tf.block<3,1>(0,3));
}

//Definition (3.20) page 98, MR pre-print 2019 - Adjoint map
Eigen::VectorXd adjoint_map(const Eigen::VectorXd &twist, const Eigen::Matrix4d &tf)
{
    //... equation from the book implemented here
    return adjoint_matrix(tf)*twist;
}

//Equation (3.70) page 96, MR pre-print 2019
Eigen::VectorXd twist(const Eigen::Vector3d &w, const Eigen::Vector3d &v)
{
    //... equation from the book implemented here
    Eigen::VectorXd tw(6);
    tw << w,v;
    return tw;
}

//Figure (3.19)(formula without reference) page 101, MR pre-print 2019
Eigen::VectorXd twist(const Eigen::Vector3d &q, const Eigen::Vector3d &s, double h, double angular_velocity)
{
    //... equation from the book implemented here
    Eigen::VectorXd tw(6);
    tw << angular_velocity*s,angular_velocity*(-s.cross(q) + h * s);
    return tw;
}

//Equation (3.71) page 96, MR pre-print 2019
Eigen::Matrix4d twist_matrix(const Eigen::Vector3d &w, const Eigen::Vector3d &v)
{
    //... equation from the book implemented here
    Eigen::Matrix4d matrix;
    matrix.block<3,3>(0,0) = skew_symmetric(w);
    matrix.block<3,1>(0,3) = v;
    matrix.block<4,1>(3,0) << 0.0,0.0,0.0,1.0;
    return matrix;
}

//Equation (3.70) page 96, MR pre-print 2019 - Twist
Eigen::Matrix4d twist_matrix(const Eigen::VectorXd &twist)
{
    //... equation from the book implemented here
    return twist_matrix(twist.block<3,1>(0,0),twist.block<3,1>(0,3));
}

//Definition (3.24) points (a) & (b) above, page 102, MR pre-print 2019
Eigen::VectorXd screw_axis(const Eigen::Vector3d &w, const Eigen::Vector3d &v)
{
    //... equation from the book implemented here
    Eigen::VectorXd screw(6);
    if (floatEquals(w.norm(),0.0)) {
        screw << 0.0,0.0,0.0,v.normalized();
    }
    else {
        screw << w.normalized(),v/w.norm();
    }
    return screw;
}

//Definition (3.24) page 102, MR pre-print 2019
Eigen::VectorXd screw_axis(const Eigen::Vector3d &q, const Eigen::Vector3d &s, double h)
{
    //... equation from the book implemented here
    Eigen::VectorXd screw(6);
    screw << s,(-s.cross(q) + h * s);
    return screw;
}

//Equation (3.51) page 82, MR pre-print 2019
Eigen::Matrix3d matrix_exponential(const Eigen::Vector3d &w, double theta)
{
    //... equation from the book implemented here
    Eigen::Matrix3d skew_w = skew_symmetric(w);
    return Eigen::Matrix3d::Identity() + std::sin(theta)*skew_w + (1.0-std::cos(theta))*skew_w*skew_w;
}

//Proposition (3.25) page 103, MR pre-print 2019
Eigen::Matrix4d matrix_exponential(const Eigen::Vector3d &w, const Eigen::Vector3d &v, double theta)
{
    //... equations from the book implemented here
    Eigen::Matrix4d matrix;
    if (floatEquals(w.norm(),0.0)) {
        matrix = transformation_matrix(Eigen::Matrix3d::Identity(),v*theta);
    }
    else{
        Eigen::Matrix3d skew_w = skew_symmetric(w);
        matrix = transformation_matrix(matrix_exponential(w, theta),(Eigen::Matrix3d::Identity()*theta + (1.0-std::cos(theta))*skew_w + (theta-std::sin(theta))*skew_w*skew_w)*v);
    }
    return matrix;
}

//Equation (3.70) page 96, MR pre-print 2019 - Twist
Eigen::Matrix4d matrix_exponential(const Eigen::VectorXd &screw, double theta)
{
    //... equations from the book implemented here
    return matrix_exponential(screw.head(3),screw.tail(3),theta);
}

//Algorithm page 85, MR pre-print 2019
std::pair<Eigen::Vector3d, double> matrix_logarithm(const Eigen::Matrix3d &r)
{
    //... equations from the book implemented here
    double t_rad = 0.0;
    Eigen::Vector3d w{0.0, 0.0, 0.0};

    if (floatEquals(-1.0, r.trace()))
    {
        t_rad = EIGEN_PI;

        if (!floatEquals(r(2,2),-1.0))
        {
            w = {r(0,2), r(1,2), 1.0 + r(2,2)}/sqrt(2.0*(1.0+r(2,2)));
        }
        else if (!floatEquals(r(1,1),-1.0))
        {
            w = {r(0,1), 1.0+ r(1,1),r(2,1)}/sqrt(2.0*(1.0+r(1,1)));
        }
        else if (!floatEquals(r(0,0),-1.0))
        {
            w = {1.0 + r(0,0),r(1,0),r(2,0)}/sqrt(2.0*(1.0+r(0,0)));
        }
        else
        {
            throw std::runtime_error("Invalid rotation matrix");
        }
    }
    else if (!r.isApprox(Eigen::Matrix3d::Identity(), equal_precision))
    {
        t_rad = 1.0/std::cos(0.5*(r(0,0)+r(1,1)+r(2,2)-1.0));
        w = from_skew_symmetric((r-r.transpose())/(2.0*std::sin(t_rad)));
    }

    return {w, t_rad};
}

//Algorithm page 104, MR pre-print 2019
std::pair<Eigen::VectorXd, double> matrix_logarithm(const Eigen::Matrix3d &r, const Eigen::Vector3d &p)
{
    //... equations from the book implemented here
    Eigen::Vector3d w{0.0,0.0,0.0};
    Eigen::Vector3d v{0.0, 0.0, 0.0};
    double t_rad = 0.0;

    if (r.isApprox(Eigen::Matrix3d::Identity(),equal_precision))
    {
        v = p.normalized();
        t_rad = p.norm();
    }
    else
    {
        std::pair<Eigen::VectorXd, double> a = matrix_logarithm(r);
        w=a.first;
        t_rad=a.second;
        Eigen::Matrix3d skew_w = skew_symmetric(w);
        v = (Eigen::Matrix3d::Identity()/t_rad - 0.5*skew_w + (1.0/t_rad - 0.5*cot(t_rad/2.0))*skew_w*skew_w)*p;
    }

    return {twist(w,v), t_rad};
}

//Definition (3.13) page 87, MR pre-print 2019
std::pair<Eigen::VectorXd, double> matrix_logarithm(const Eigen::Matrix4d &tf)
{
    //... equations from the book implemented here
    return matrix_logarithm(tf.block<3,3>(0,0),tf.block<3,1>(0,3));
}

//Formula without reference page 72, Rot(x,theta), MR pre-print 2019
Eigen::Matrix3d rotate_x(double radians)
{
    //... equation from the book implemented here
    Eigen::Matrix3d matrix;
    matrix << 1.0, 0.0, 0.0,
    0.0, std::cos(radians), -std::sin(radians),
    0.0, std::sin(radians), std::cos(radians);
    return matrix;
}

//Formula without reference page 72, Rot(y,theta), MR pre-print 2019
Eigen::Matrix3d rotate_y(double radians)
{
    //... equation from the book implemented here
    Eigen::Matrix3d matrix;
    matrix << std::cos(radians), 0.0, std::sin(radians),
    0.0, 1.0, 0.0,
    -std::sin(radians), 0.0, std::cos(radians);
    return matrix;
}

//Formula without reference page 72, Rot(z,theta), MR pre-print 2019
Eigen::Matrix3d rotate_z(double radians)
{
    //... equation from the book implemented here
    Eigen::Matrix3d matrix;
    matrix << std::cos(radians), -std::sin(radians), 0.0,
    std::sin(radians), std::cos(radians), 0.0,
    0.0, 0.0, 1.0;
    return matrix;
}

//Equations (3.13), (3.14), (3.15) and (3.16), page 65, MR pre-print 2019
Eigen::Matrix3d rotation_matrix_from_frame_axes(const Eigen::Vector3d &x, const Eigen::Vector3d &y, const Eigen::Vector3d &z)
{
    //... equations from the book implemented here
    Eigen::Matrix3d matrix;
    matrix.block<3,1>(0,0) = x;
    matrix.block<3,1>(0,1) = y;
    matrix.block<3,1>(0,2) = z;
    return matrix;
}

//Formula without reference B.1 Euler Angles, R(alpha,beta,gamma), page 577, Rot(w,theta), MR pre-print 2019
Eigen::Matrix3d rotation_matrix_from_euler_zyx(const Eigen::Vector3d &e)
{
    //... equation from the book implemented here
    return rotate_z(e(0))*rotate_y(e(1))*rotate_x(e(2));
}

//Formula without reference page 72, Rot(w,theta), MR pre-print 2019
Eigen::Matrix3d rotation_matrix_from_axis_angle(const Eigen::Vector3d &axis, double radians)
{
    //... equation from the book implemented here
    Eigen::Matrix3d matrix;
    double c = std::cos(radians);
    double s = std::sin(radians);
    double w1 = axis(0);
    double w2 = axis(1);
    double w3 = axis(2);
    matrix << c+(w1*w1)*(1.0-c), w1*w2*(1.0-c)-w3*s, w1*w3*(1.0-c)+w2*s,
    w1*w2*(1.0-c)+w3*s, c+(w2*w2)*(1.0-c), w2*w3*(1.0-c)-w1*s,
    w1*w3*(1.0-c)-w2*s, w2*w3*(1.0-c)+w1*s, c+(w3*w3)*(1.0-c);
    return matrix;
}

//Definition (3.13) page 87, MR pre-print 2019
Eigen::Matrix3d rotation_matrix(const Eigen::Matrix4d &tf)
{
    //... equation from the book implemented here
    return tf.block<3,3>(0,0);
}

//Definition (3.13) page 87, MR pre-print 2019
Eigen::Matrix4d transformation_matrix(const Eigen::Vector3d &p)
{
    //... equation from the book implemented here
    Eigen::Matrix4d matrix;
    matrix.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
    matrix.block<3,1>(0,3) = p;
    matrix.block<1,4>(3,0) << 0.0,0.0,0.0,1.0;
    return matrix;
}

//Definition (3.13) page 87, MR pre-print 2019
Eigen::Matrix4d transformation_matrix(const Eigen::Matrix3d &r)
{
    //... equation from the book implemented here
    Eigen::Matrix4d matrix;
    matrix.block<3,3>(0,0) = r;
    matrix.block<3,1>(0,3) << 0.0,0.0,0.0;
    matrix.block<1,4>(3,0) << 0.0,0.0,0.0,1.0;
    return matrix;
}

//Definition (3.13) page 87, MR pre-print 2019
Eigen::Matrix4d transformation_matrix(const Eigen::Matrix3d &r, const Eigen::Vector3d &p)
{
    //... equation from the book implemented here
    Eigen::Matrix4d matrix;
    matrix.block<3,3>(0,0) = r;
    matrix.block<3,1>(0,3) = p;
    matrix.block<1,4>(3,0) << 0.0,0.0,0.0,1.0;
    return matrix;
}
}
