#include <iostream>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

int main(int argc, char** argv)
{
    // Get rotation matrix from rotation vector: from .matrix()
    Matrix3d rotation_matrix = Matrix3d::Identity();
    AngleAxisd rotation_vector(M_PI / 4, Vector3d(0, 0, 1));
    cout.precision(3);
    cout << "rotation matrix = \n" << rotation_vector.matrix() << endl;

    // Get rotation matrix from rotation vector: from .toRotationMatrix()
    rotation_matrix = rotation_vector.toRotationMatrix();

    // Transform with AngleAxis
    Vector3d v(1, 0, 0);
    Vector3d v_rotated = rotation_vector * v;
    cout << "(1, 0, 0) after rotation (by angle axis) = " << v_rotated.transpose() << endl;

    // Get euler angle
    Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0);
    cout << "yaw pitch roll = " << euler_angles.transpose() << endl;

    // Get homegeneous transform matrix
    Isometry3d T = Isometry3d::Identity();
    T.rotate(rotation_matrix);
    T.pretranslate(Vector3d(1, 3, 4));
    cout << "Transform matrix = \n" << T.matrix() << endl;

    // Apply homegeneous transform
    Vector3d v_transformed = T * v;
    cout << "v transformed = \n" << v_transformed.transpose() << endl;

    // Quaternion
    // Get quaternion from rotation vector
    Quaterniond q = Quaterniond(rotation_vector);
    cout << "quaternion from rotation vector = " << q.coeffs().transpose() << endl;

    // Get quaternion from rotation matrix
    q = Quaterniond(rotation_matrix);
    cout << "quaternion from rotation matrix = " << q.coeffs().transpose() << endl;

    // Use quaternion to rotate: with overloaded multiplication operator
    v_rotated = q * v;
    cout << "(1, 0, 0) after rotation = " << v_rotated.transpose() << endl;

    // Use quaternion to rotate: its regular representation
    cout << "which is equivalent to "
         << (q * Quaterniond(0, 1, 0, 0) * q.inverse()).coeffs().transpose() << endl;

    return 0;
}
