//
// Created by xiang on 12/21/17.
//

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>

#include "sophus/se3.h"

using namespace std;

typedef vector<Vector3d, Eigen::aligned_allocator<Vector3d>> VecVector3d;
typedef vector<Vector2d, Eigen::aligned_allocator<Vector2d>> VecVector2d;
typedef Matrix<double, 6, 1> Vector6d;

string p3d_file = "./p3d.txt";
string p2d_file = "./p2d.txt";

int main(int argc, char **argv) {

    VecVector2d p2d;
    VecVector3d p3d;
    Matrix3d K;
    double fx = 520.9, fy = 521.0, cx = 325.1, cy = 249.7;
    K << fx, 0, cx, 0, fy, cy, 0, 0, 1;
    cout<<K<<endl;
    // load points in to p3d and p2d 
    // implement pose reading code
    // start your code here (5~10 lines)
    // START YOUR CODE HERE
    ifstream inFILE_2(p2d_file),inFILE_3(p3d_file);
    double u,v,x,y,z;;
    while(!inFILE_2.eof())
    {
        inFILE_2>>u;
        inFILE_2>>v;
        p2d.push_back(Eigen::Vector2d(u,v));
    }
    cout<<"read p2d_file finished!"<<endl;
    while(!inFILE_3.eof())
    {
        inFILE_3>>x;
        inFILE_3>>y;
        inFILE_3>>z;
        p3d.push_back(Eigen::Vector3d(x,y,z));
    }
    // END YOUR CODE HERE
    cout<<"read p3d_file finished!"<<endl;
    assert(p3d.size() == p2d.size());

    int iterations = 100;
    double cost = 0, lastCost = 0;
    int nPoints = p3d.size();
    cout << "points: " << nPoints << endl;

    Sophus::SE3 T_esti; // estimated pose 待估计的位姿

    for (int iter = 0; iter < iterations; iter++) {

        Matrix<double, 6, 6> H = Matrix<double, 6, 6>::Zero();
        Vector6d b = Vector6d::Zero();

        cost = 0;
        // compute cost

//        double s = 5000.0;
        for (int i = 0; i < nPoints; i++) {
            // compute cost for p3d[I] and p2d[I]
            // START YOUR CODE HERE
//            P_e.x() = fx*p3d[i].x()/p3d[i].z()+cx;
//            P_e.y() = fy*p3d[i].y()/p3d[i].z()+cy;
            Vector3d P = T_esti.rotation_matrix()*p3d[i]+T_esti.translation(); //计算变换之后的三维空间点的坐标
//            Vector3d P = T_esti*p3d[i];
            double x = P[0];
            double y = P[1];
            double z = P[2];
            Vector3d P_uv = K*P/z; //将三维空间点投影到归一化的相机平面上
            Vector2d e(p2d[i].x()-P_uv.x(),p2d[i].y()-P_uv.y()); //误差向量
            cost += e.transpose()*e;
            // END YOUR CODE HERE

            // compute jacobian
            Matrix<double, 2, 6> J;
            // START YOUR CODE HERE

            J(0,0) = fx/z;
            J(0,1) = 0;
            J(0,2) = -fx*x/(z*z);

            J(0,3) = -fx*x*y/(z*z);
            J(0,4) = fx+fx*x*x/(z*z);
            J(0,5) = -fx*y/z;

            J(1,0) = 0;
            J(1,1) = fy/z;
            J(1,2) = -fy*y/(z*z);

            J(1,3) = -fy-fy*y*y/(z*z);
            J(1,4) = fy*x*y/(z*z);
            J(1,5) = fy*x/z;
            J = -J;

            // END YOUR CODE HERE

            H += J.transpose() * J;
            b += -J.transpose() * e;
        }

	    // solve dx 
        Vector6d dx;
        // START YOUR CODE HERE 
        dx = H.ldlt().solve(b);
        // END YOUR CODE HERE

        if (isnan(dx[0])) {
            cout << "result is nan!" << endl;
            break;
        }

        if (iter > 0 && cost >= lastCost) {
            // cost increase, update is not good
            cout << "cost: " << cost << ", last cost: " << lastCost << endl;
            break;
        }

        // update your estimation
        // START YOUR CODE HERE 
        T_esti *=  Sophus::SE3::exp(dx); //更新估计
//        T_esti =  Sophus::SE3::exp(dx) * T_esti; //更新估计
        // END YOUR CODE HERE
        
        lastCost = cost;

        cout << "iteration " << iter << " cost=" << cout.precision(12) << cost << endl;
    }

    cout << "estimated pose: \n" << T_esti.matrix() << endl;
    return 0;
}
