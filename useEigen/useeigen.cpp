#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Householder>
#include <eigen3/Eigen/Cholesky>
#include<random>
#include <ctime>
#include<iostream>

using namespace std;
using namespace Eigen;

// double generate_random(double dummy){
//   static default_random_engine e(time(0));
//   static normal_distribution<double> n(0,10);
//   return n(e);
// }

int main()
{
    // A.unaryExpr([](double ele){return rand() / double(RAND_MAX);});
    // A.unaryExpr([](double dummy){return n(e);});
    // A.unaryExpr(ptr_fun(generate_random));

    // 对A,b初始化;
    int rows = 3;
    int cols = 3;
    static default_random_engine e(time(0));
    static normal_distribution<double> n(0,10);
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> A(rows,cols);
    Eigen::Matrix<double,Eigen::Dynamic,1> b(rows,1);
    // A.unaryExpr([](double dummy){return abs(n(e));});
    // b.unaryExpr([](double dummy){return n(e);});
    for(int x = 0;x<rows;x++)
    {
        b(x,0) = random()%10;
        for(int y = 0;y<cols;y++)
        {
            A(x,y) = abs(n(e));
            // cout<<A(x,y)<<" ";
        }
        // cout<<endl;
    }
    A = A*A.transpose();
    cout<<"A:"<<endl<<A<<endl;
    cout<<"b:"<<endl<<b<<endl;

    cout<<"orginal: x "<<endl<<A.inverse()*b<<endl;
    // QR分解
    // 求Ax=b | Ax = QRx = b | Rx = QTb 
    // 1. 原版
    Eigen::Matrix<double,Eigen::Dynamic,1> x_qr(rows,1);
    x_qr = A.colPivHouseholderQr().solve(b);
    cout<<"original: x_qr:"<<endl<<x_qr<<endl;

    // 2. 分解版本
	HouseholderQR<Matrix<double,Eigen::Dynamic,Eigen::Dynamic>> qr;
	qr.compute(A);
	MatrixXd R = qr.matrixQR().triangularView<Eigen::Upper>();
	MatrixXd Q = qr.householderQ();
    x_qr = R.inverse()*Q.transpose()*b;
    cout<<"v1 : x_qr:"<<endl<<x_qr<<endl;

    // Cholesky分解
    // Ax=b; L*L^T x = b;x
    Eigen::Matrix<double,Eigen::Dynamic,1> x_cholesky1(rows,1);
    x_cholesky1 = A.llt().solve(b);
    cout<<"original : x_cholesky1:"<<endl<<x_cholesky1<<endl;

    Eigen::Matrix<double,Eigen::Dynamic,1> x_cholesky2(rows,1);
    x_cholesky2 = A.ldlt().solve(b);
    cout<<"original : x_cholesky2:"<<endl<<x_cholesky2<<endl;


    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> L(rows,cols);
    L = A.llt().matrixL();
    // cout<<"L:"<<endl<<L<<endl;
    // cout<<"L*LT:"<<endl<<L*L.transpose()<<endl;
    // x_cholesky = b*L.transpose()*L;   
}