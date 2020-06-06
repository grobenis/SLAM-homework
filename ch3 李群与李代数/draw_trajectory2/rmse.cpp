//
// Created by guoben on 2020/6/1.
//

#include <string>
#include <iostream>
#include <fstream>
#include <unistd.h>

// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>
#include <sophus/se3.h>

using namespace std;

// path to trajectory file
string ground_truth_file = "groundtruth.txt";
string estimated_file = "estimated.txt";

// function for plotting trajectory, don't edit this code
// start point is red and end point is blue
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>);
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> truth_poses,vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> estimated_poses);

double calculateRMSE(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> truth_poses,vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> estimated_poses);
int main(int argc, char **argv) {

    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> truth_poses;
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> estimated_poses;
    ifstream truthFILE(ground_truth_file);
    if (!truthFILE.is_open())
    {
        cout << "未成功打开文件" << endl;
    }
    // implement pose reading code
    double t,t_x,t_y,t_z,q_x,q_y,q_z,q_w;
    while(!truthFILE.eof())
    {
        truthFILE>>t;
        truthFILE>>t_x;
        truthFILE>>t_y;
        truthFILE>>t_z;
        truthFILE>>q_x;
        truthFILE>>q_y;
        truthFILE>>q_z;
        truthFILE>>q_w;
        truth_poses.push_back(Sophus::SE3(Eigen::Quaterniond(q_w,q_x,q_y,q_z),Eigen::Vector3d(t_x,t_y,t_z)));
    }
    // end your code here

    ifstream ESTIMATEDFILE(estimated_file);
    if (!ESTIMATEDFILE.is_open())
    {
        cout << "未成功打开文件" << endl;
    }
    // implement pose reading code
    // start your code here (5~10 lines)
    while(!ESTIMATEDFILE.eof())
    {
        ESTIMATEDFILE>>t;
        ESTIMATEDFILE>>t_x;
        ESTIMATEDFILE>>t_y;
        ESTIMATEDFILE>>t_z;
        ESTIMATEDFILE>>q_x;
        ESTIMATEDFILE>>q_y;
        ESTIMATEDFILE>>q_z;
        ESTIMATEDFILE>>q_w;
        estimated_poses.push_back(Sophus::SE3(Eigen::Quaterniond(q_w,q_x,q_y,q_z),Eigen::Vector3d(t_x,t_y,t_z)));
    }

    double rmse = calculateRMSE(truth_poses,estimated_poses);
    cout<<"rmse:"<<rmse<<endl;
    // draw trajectory in pangolin
//    DrawTrajectory(truth_poses);
//    DrawTrajectory(estimated_poses);
    DrawTrajectory(truth_poses,estimated_poses);
    return 0;
}

/*******************************************************************************************/
double calculateRMSE(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> truth_poses,vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> estimated_poses)
{
    double rmse=0.0;
    for(int i = 0;i<truth_poses.size();i++)
    {
//        Eigen::Vector3d t(truth_poses[i].translation()-estimated_poses[i].translation());
//        Eigen::Quaterniond q(truth_poses[i].unit_quaternion()*estimated_poses[i].unit_quaternion().inverse());
//        sum+=t.squaredNorm()+q.squaredNorm();
        Eigen::Matrix<double ,6,1> se3;
        se3 = (truth_poses[i].inverse()*estimated_poses[i]).log();

        rmse+=se3.squaredNorm();
    }
//    double rmse = (double)sqrt(sum);
    rmse = sqrt(rmse/(double)truth_poses.size());
    return rmse;
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

void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses) {
    if (poses.empty()) {
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
        for (size_t i = 0; i < poses.size() - 1; i++) {
            glColor3f(1 - (float) i / poses.size(), 0.0f, (float) i / poses.size());
//           cout<<(float)i/poses.size()<<endl;
            glBegin(GL_LINES);
            auto p1 = poses[i], p2 = poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }

}