//
// Created by guoben on 2020/6/18.
//

#include "iostream"
#include "fstream"
#include "sophus/se3.h"
#include "Eigen/Eigen"
#include <unistd.h>
#include <pangolin/pangolin.h>
using namespace std;
using namespace Eigen;
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> truth_poses,vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> estimated_poses);
int main()
{
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> esti_poses;
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> ground_truth_poses;
    //读取文件
    fstream INFILE("compare.txt");
    double time,t_x,t_y,t_z,q_x,q_y,q_z,q_w;
    while(!INFILE.eof())
    {
        INFILE>>time;
        INFILE>>t_x;
        INFILE>>t_y;
        INFILE>>t_z;
        INFILE>>q_x;
        INFILE>>q_y;
        INFILE>>q_z;
        INFILE>>q_w;
        esti_poses.push_back(Sophus::SE3(Eigen::Quaterniond(q_w,q_x,q_y,q_z),Eigen::Vector3d(t_x,t_y,t_z)));
        INFILE>>time;
        INFILE>>t_x;
        INFILE>>t_y;
        INFILE>>t_z;
        INFILE>>q_x;
        INFILE>>q_y;
        INFILE>>q_z;
        INFILE>>q_w;
        ground_truth_poses.push_back(Sophus::SE3(Eigen::Quaterniond(q_w,q_x,q_y,q_z),Eigen::Vector3d(t_x,t_y,t_z)));
    }
    assert(esti_poses.size()==ground_truth_poses.size());
    int s=esti_poses.size();
    cout<<esti_poses.size()<<' '<<ground_truth_poses.size()<<endl;

    //1. 质心计算
    Eigen::Vector3d center_et(0,0,0);//估计出轨迹的质心
    Eigen::Vector3d center_gd(0,0,0);//groundtruth的质心
    for(int i = 0;i<esti_poses.size();i++)
    {
        center_et+=esti_poses[i].translation();
        center_gd+=ground_truth_poses[i].translation();
    }
    center_et /= s;
    center_gd /= s;
    // 2. 计算去质心坐标
    vector<Vector3d> t_esti;
    vector<Vector3d> t_gd;
    for(int i = 0;i<s;i++)
    {
        t_esti.push_back(esti_poses[i].translation()-center_et);
        t_gd.push_back(ground_truth_poses[i].translation()-center_gd);
    }

    //3. 根据优化问题计算旋转矩阵R （使用最小二乘法迭代计算R）
    Matrix3d R; //待估计R
    Matrix3d W;
    W.setZero();
    cout<<W<<endl;
    //A.SVD方法
    for(int i = 0;i<s;i++)
        W+=t_gd[i]*t_esti[i].transpose();
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W,ComputeThinU|ComputeThinV);
    Matrix3d V=svd.matrixV(),U=svd.matrixU();
    R=U*V.transpose();

    cout<<"R:"<<endl<<R<<endl;
////    B.最优化方法
//    int iterations = 100; //最大迭代次数
//    for (int iter = 0; iter < iterations; iter++) {
//        double cost = 0;
//        for(int i = 0; i<s;i++)
//        {
//            Vector3d error = t_gd[i]-R*t_esti[i];
//            cost+=error.transpose() *error;
//        }
//    }
//    cout<<"R:"<<endl<<R<<endl;

    //3. 根据R计算t
    Vector3d t;
    t = center_gd - R*center_et;
    cout<<"t:"<<t<<endl;

    //画出轨迹
    cout<<"red:truth"<<endl;
    cout<<"blue::esti"<<endl;
//    DrawTrajectory(ground_truth_poses,esti_poses);

//    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> esti_poses_rt;
//    for(int i = 0;i<s;i++)
//    {
//        Sophus::SE3 p_rt;
//        p_rt = Sophus::SE3(R,t)*esti_poses[i];
//        esti_poses_rt.push_back(p_rt);
//    }
//    DrawTrajectory(ground_truth_poses,esti_poses_rt);
//
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> truth_poses_rt;
    R=R.inverse();
    t=- R*t;
    for(int i = 0;i<s;i++)
    {
        Sophus::SE3 p_rt;
        p_rt = Sophus::SE3(R,t)*ground_truth_poses[i];
        truth_poses_rt.push_back(p_rt);
    }
    DrawTrajectory(truth_poses_rt,esti_poses);
    return 0;
}

void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> truth_poses,vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> estimated_poses){
    if (truth_poses.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < truth_poses.size() - 1; i++) {
//            glColor3f(1 - (float) i / truth_poses.size(), 0.0f, (float) i / truth_poses.size());
            glColor3f(1.0f, 0.0f, 0.0f);
            glBegin(GL_LINES);
            auto p1 = truth_poses[i], p2 = truth_poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();

            glColor3f(0.0f, 0.0f, 1.0f);
            glBegin(GL_LINES);
            p1 = estimated_poses[i], p2 = estimated_poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();

        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}