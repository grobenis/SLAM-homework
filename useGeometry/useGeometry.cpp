#include <eigen3/Eigen/Geometry>
#include<iostream>

using namespace std;
using namespace Eigen;

int main()
{
    //机器人一号的位置姿态
    Quaterniond q1(0.55,0.3,0.2,0.2);
    Vector3d t1(0.7,1.1,0.2);
    //机器人二号的位置姿态
    Quaterniond q2(-0.1,0.3,-0.7,0.2);
    Vector3d t2(-0.1,0.4,0.8);

    //四元数的归一化
    q1.normalize();
    q2.normalize();
	
    Vector3d p1(0.5,-0.1,0.2);
    Vector4d p1a;
    p1a<<p1,1;
    
    //构建T_cw_1
    Matrix4d T_cw_1;
    T_cw_1 << q1.toRotationMatrix() ,t1,
              0,0,0,1;
    cout<<"T_cw_1:"<<endl<<T_cw_1<<endl;

    //构建T_cw_2
    Matrix4d T_cw_2;
    T_cw_2 <<q2.toRotationMatrix(),t2,
            0,0,0,1;
    cout<<"T_cw_2:"<<endl<<T_cw_2<<endl;

    Vector4d p2a;
    // p2a = T_cw_2.inverse() * T_cw_1 *p1a;
    p2a = T_cw_2 * T_cw_1.inverse() *p1a;
    
    Vector3d p2;
    p2 = p2a.block(0,0,3,1);
    cout<<"p2:"<<endl<<p2<<endl;
    return 0;
}