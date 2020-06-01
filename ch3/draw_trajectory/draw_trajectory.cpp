
#include <pangolin/pangolin.h>

int main(  ) {
    //生成一个gui界面，定义大小
    pangolin::CreateWindowAndBind("Main", 640, 480);
    //进行深度测试，保证某视角下像素只有一种颜色，不混杂
    glEnable(GL_DEPTH_TEST);

    //放置一个相机
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.2, 100),
            pangolin::ModelViewLookAt(-2, 2, -2, 0, 0, 0, pangolin::AxisY)
    );

    //创建视角窗口
    pangolin::Handler3D handler(s_cam);
    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f / 480.0f)
            .SetHandler(&handler);

    while (!pangolin::ShouldQuit()) {
        //清除颜色缓冲和深度缓冲
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);

        //绘制立方体
        pangolin::glDrawColouredCube();

        //绘制坐标系  线宽 启动 颜色 起点 终点 停止
        glLineWidth(3);
        glBegin(GL_LINES);
        glColor3f(0.8f, 0.f, 0.f);
        glVertex3f(-1, -1, -1);
        glVertex3f(0, -1, -1);
        glColor3f(0.f, 0.8f, 0.f);
        glVertex3f(-1, -1, -1);
        glVertex3f(-1, 0, -1);
        glColor3f(0.2f, 0.2f, 1.f);
        glVertex3f(-1, -1, -1);
        glVertex3f(-1, -1, 0);
        glEnd();

        //交换帧和并推进事件
        pangolin::FinishFrame();
    }

    return 0;
}
//#include <sophus/se3.h>
//#include <string>
//#include <iostream>
//#include <fstream>
//#include <unistd.h>
//
//// need pangolin for plotting trajectory
////#include <pangolin/pangolin.h>
//
//using namespace std;
//
//// path to trajectory file
//string trajectory_file = "./trajectory.txt";
//
//// function for plotting trajectory, don't edit this code
//// start point is red and end point is blue
////void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>);
//int main(int argc, char **argv) {
//
//    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses;
//
//    /// implement pose reading code
//    // start your code here (5~10 lines)
//
//    // end your code here
//
//    // draw trajectory in pangolin
////    DrawTrajectory(poses);
//    return 0;
//}
//
///*******************************************************************************************/
////void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses) {
////    if (poses.empty()) {
////        cerr << "Trajectory is empty!" << endl;
////        return;
////    }
////
////    // create pangolin window and plot the trajectory
////    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
////    glEnable(GL_DEPTH_TEST);
////    glEnable(GL_BLEND);
////    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
////
////    pangolin::OpenGlRenderState s_cam(
////            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
////            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
////    );
////
////    pangolin::View &d_cam = pangolin::CreateDisplay()
////            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
////            .SetHandler(new pangolin::Handler3D(s_cam));
////
////
////    while (pangolin::ShouldQuit() == false) {
////        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
////
////        d_cam.Activate(s_cam);
////        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
////
////        glLineWidth(2);
////        for (size_t i = 0; i < poses.size() - 1; i++) {
////            glColor3f(1 - (float) i / poses.size(), 0.0f, (float) i / poses.size());
////            glBegin(GL_LINES);
////            auto p1 = poses[i], p2 = poses[i + 1];
////            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
////            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
////            glEnd();
////        }
////        pangolin::FinishFrame();
////        usleep(5000);   // sleep 5 ms
////    }
////
////}