#include <iostream>
#include <sophus/so3.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>

using namespace std;

int main()
{
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
    Eigen::Quaterniond q(R);

    // use roation matrix to update
    Eigen::Vector3d w(0.01, 0.02, 0.03);
    Eigen::Matrix3d w_hat;
    w_hat << 0, -w(2), w(1),
            w(2), 0, -w(0),
            -w(1), w(0), 0;
    Eigen::Matrix3d delt_r = w_hat.exp();
    Eigen::Matrix3d update_R = R*delt_r;


    // use quaternion to update
    Eigen::Quaterniond delt_q(1, 0.5*w(0), 0.5*w(1), 0.5*w(2));
    Eigen::Quaterniond update_Q = q*delt_q;
    cout << "new q1= " << endl << Eigen::Quaterniond(update_R).coeffs() << endl;
    cout << "new q2= " << endl << update_Q.coeffs() << endl;


    //use sopus to update
    Sophus::SO3 so3(R);
    Sophus::SO3 update_so3 = so3*Sophus::SO3::exp(w);
    cout << "new so3_1 =" << endl << Sophus::Quaterniond(update_so3.matrix()).coeffs() << endl;

    return 0;
}