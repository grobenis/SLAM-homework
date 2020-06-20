//
// Created by 高翔 on 2017/12/19.
// 本程序演示如何从Essential矩阵计算R,t
//

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;

#include <sophus/so3.h>

#include <iostream>

using namespace std;

int main(int argc, char **argv) {

    // 给定Essential矩阵
    Matrix3d E;
    E << -0.0203618550523477, -0.4007110038118445, -0.03324074249824097,
            0.3939270778216369, -0.03506401846698079, 0.5857110303721015,
            -0.006788487241438284, -0.5815434272915686, -0.01438258684486258;

    // 待计算的R,t
    Matrix3d R;
    Vector3d t;

    // SVD and fix sigular values
    // START YOUR CODE HERE
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(E,ComputeThinU|ComputeThinV);
    Matrix3d V=svd.matrixV(),U=svd.matrixU();
    Matrix3d un_S=U.inverse()* E*V.transpose().inverse(); //类型不要搞混
    //计算后的Sigma矩阵
    double delta_1=un_S(0,0),delta_2=un_S(1,1);
    Matrix3d S = Matrix3d::Zero();
    S(0,0)=(delta_1+delta_2)/2;
    S(1,1)=(delta_1+delta_2)/2;
    // END YOUR CODE HERE

    // set t1, t2, R1, R2 
    // START YOUR CODE HERE
    Matrix3d t_wedge1;
    Matrix3d t_wedge2;

    Matrix3d R1;
    Matrix3d R2;
    // END YOUR CODE HERE
    //1.使用旋转的角度和旋转轴向量（此向量为单位向量）来初始化角轴
    AngleAxisd V1(M_PI / 2, Vector3d(0, 0, 1));
    AngleAxisd V2(- M_PI / 2, Vector3d(0, 0, 1));

    Matrix3d Rz_pos = V1.toRotationMatrix();
    Matrix3d Rz_neg = V2.toRotationMatrix();
    t_wedge1 = U*Rz_pos*S*U.transpose();
    t_wedge2 = U*Rz_neg*S*U.transpose();
    R1 = U*Rz_pos*V.transpose();
    R2 = U*Rz_neg*V.transpose();

    cout << "R1 = " << R1 << endl;
    cout << "R2 = " << R2 << endl;
    cout << "t1 = " << Sophus::SO3::vee(t_wedge1) << endl;
    cout << "t2 = " << Sophus::SO3::vee(t_wedge2) << endl;

    // check t^R=E up to scale
    Matrix3d tR = t_wedge1 * R1;
    cout << "t^R = " << tR << endl;
    return 0;
}