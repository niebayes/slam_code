#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

int main(int argc, char** argv)
{
    Quaterniond q1(0.35, 0.2, 0.3, 0.1), q2(-0.5, 0.4, -0.1, 0.2);
    Vector3d t1(0.3, 0.1, 0.1), t2(-0.1, 0.5, 0.3);

    q1.normalize();
    q2.normalize();

    Isometry3d T1(q1), T2(q2);
    T1.pretranslate(t1);
    T2.pretranslate(t2);

    Vector3d p1(0.5, 0, 0.2);

    Vector3d p2 = T2 * T1.inverse() * p1;

    cout << p2 << endl;

    return 0;
}
