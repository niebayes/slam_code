#include <iostream>
#include <string>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "../3rdparty/Sophus/sophus/se3.hpp"

using namespace std;
using namespace Eigen;

int main(int argc, char** argv)
{
    // Create a rotation matrix rotating 90 degrees around z axis
    Matrix3d R = AngleAxisd(M_PI / 2, Vector3d(0, 0, 1)).toRotationMatrix();
    // Get its corresponding quaternion
    Quaterniond q(R);

    // Construct SO3 from rotation matrix or quaternion
    Sophus::SO3d SO3_R(R);
    Sophus::SO3d SO3_q(q);
    cout << "SO(3) from rotation matrix: \n" << SO3_R.matrix() << endl;
    cout << "SO(3) from quaternion: \n" << SO3_q.matrix() << endl;
    cout << "they are equal" << endl;

    // Use log map to get Lie algebra
    Vector3d so3 = SO3_R.log();
    cout << "so3 = " << so3.transpose() << endl;

    // Get skew symmetric matrix from vector
    cout << "so3 hat = \n" << Sophus::SO3d::hat(so3) << endl;
    // Inverse Operation: Get vector from skew symmetric matrix
    cout << "so3 hat vee = " << Sophus::SO3d::vee(Sophus::SO3d::hat(so3)).transpose() << endl;

    // Update disturbance model on SO3
    Vector3d update_so3(1e-4, 0, 0);
    Sophus::SO3d SO3_updated = Sophus::SO3d::exp(update_so3) * SO3_R;
    cout << "SO3 updated = \n" << SO3_updated.matrix() << endl;

    // ********************************************************
    cout << string(40, '*') << endl;
    // The operations on SE3 is very similar to that on SO3
    Vector3d t(1, 0, 0);
    Sophus::SE3d SE3_Rt(R, t);
    Sophus::SE3d SE3_qt(q, t);
    cout << "SE3 from R, t = \n" << SE3_Rt.matrix() << endl;
    cout << "SE3 from q, t = \n" << SE3_qt.matrix() << endl;

    // using declaration for the sake of concise;
    using Vector6d = Eigen::Matrix<double, 6, 1>;

    // Use log map to get Lie algebra
    Vector6d se3 = SE3_Rt.log();
    cout << "se3 = " << se3.transpose() << endl;

    // Get skew symmetric matrix from vector
    cout << "se3 hat = \n" << Sophus::SE3d::hat(se3) << endl;
    // Inverse Operation: Get vector from skew symmetric matrix
    cout << "se3 hat vee = " << Sophus::SE3d::vee(Sophus::SE3d::hat(se3)).transpose() << endl;

    // Update disturbance model on SE3
    Vector6d update_se3;
    update_se3.setZero();
    update_se3(0, 0) = 1e-4;
    Sophus::SE3d SE3_updated = Sophus::SE3d::exp(update_se3) * SE3_Rt;
    cout << "SE3 updated = \n" << SE3_updated.matrix() << endl;

    return 0;
}