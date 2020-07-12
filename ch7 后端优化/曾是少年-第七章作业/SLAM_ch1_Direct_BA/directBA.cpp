//
// Created by xiang on 1/4/18.
// this program shows how to perform direct bundle adjustment
//
#include <iostream>

using namespace std;

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <opencv2/opencv.hpp>

#include <pangolin/pangolin.h>
#include <boost/format.hpp>

using namespace Eigen;

typedef vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> VecSE3;
typedef vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecVec3d;

// global variables 
// 全局变量
string pose_file = "/home/guoben/Project/Curse/SLAM_ch1_Direct_BA/poses.txt";
string points_file = "/home/guoben/Project/Curse/SLAM_ch1_Direct_BA/points.txt";

// intrinsics
// 内参
float fx = 277.34;
float fy = 291.402;
float cx = 312.234;
float cy = 239.777;

// bilinear interpolation
// 双线性差值
inline float GetPixelValue(const cv::Mat &img, float x, float y) {
    uchar *data = &img.data[int(y) * img.step + int(x)];
    float xx = x - floor(x);
    float yy = y - floor(y);
    return float(
            (1 - xx) * (1 - yy) * data[0] +
            xx * (1 - yy) * data[1] +
            (1 - xx) * yy * data[img.step] +
            xx * yy * data[img.step + 1]
    );
}

// g2o vertex that use sophus::SE3 as pose
// 定义g2o的顶点 使用SE3作为相机pose
class VertexSophus : public g2o::BaseVertex<6, Sophus::SE3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexSophus() {}

    ~VertexSophus() {}

    bool read(std::istream &is) {}

    bool write(std::ostream &os) const {}

    virtual void setToOriginImpl() {
        _estimate = Sophus::SE3d();
    }
    //更新相机pose
    virtual void oplusImpl(const double *update_) {
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> update(update_);
        setEstimate(Sophus::SE3d::exp(update) * estimate());
    }
};

// TODO edge of projection error, implement it
// 16x1 error, which is the errors in patch
// 误差模型 边的定义
typedef Eigen::Matrix<double,16,1> Vector16d; 
class EdgeDirectProjection : public g2o::BaseBinaryEdge<16, Vector16d, g2o::VertexSBAPointXYZ, VertexSophus> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeDirectProjection(float *color, cv::Mat &target) {
        this->origColor = color;
        this->targetImg = target;
    }

    ~EdgeDirectProjection() {}

    virtual void computeError() override {
        // TODO START YOUR CODE HERE
        // compute projection error ...
        const g2o::VertexSBAPointXYZ* vertexPw=static_cast<const g2o::VertexSBAPointXYZ*> (vertex(0));
        const VertexSophus* vertexTcw = static_cast<const VertexSophus*>(vertex(1));
        Eigen::Vector3d Pc = vertexTcw->estimate()*vertexPw->estimate();
        float u=Pc[0]/Pc[2]*fx+cx;
        float v=Pc[1]/Pc[2]*fy+cy;

        if(u-3<0||(u+2)> targetImg.cols||(v-3)<0||(v+2)>targetImg.rows)
        {
            _error(0,0)=0.0;
            this->setLevel(1);
        }
        else
        {
            for(int i = -2;i<2;i++)
                for(int j=-2;j<2;j++)
                {
                    int num = 4*i+j+10;
                    _error[num]= origColor[num]-GetPixelValue(targetImg,u+i,v+j);
                }
        }

        // END YOUR CODE HERE
    }

    // Let g2o compute jacobian for you
    virtual void linearizeOplus() override
    {
        if(level()==1)
        {
            _jacobianOplusXi = Matrix<double,16,3>::Zero();
            _jacobianOplusXj = Matrix<double,16,6>::Zero();
            return;
        }
        const g2o::VertexSBAPointXYZ* vertexPw = static_cast<const g2o::VertexSBAPointXYZ* >(vertex(0));
        const VertexSophus* vertexTcw = static_cast<const VertexSophus* >(vertex(1));
        Vector3d Pc = vertexTcw->estimate() * vertexPw->estimate();
        float x = Pc[0];
        float y = Pc[1];
        float z = Pc[2];
        float inv_z = 1.0/z;
        float inv_z2 = inv_z * inv_z;
        float u = x * inv_z * fx + cx;
        float v = y * inv_z * fy + cy;
      
        //u，v分别对x,y,z求导
        Matrix<double,2,3> J_Puv_Pc;
        J_Puv_Pc(0,0) = fx * inv_z;
        J_Puv_Pc(0,1) = 0;
        J_Puv_Pc(0,2) = -fx * x * inv_z2;
        J_Puv_Pc(1,0) = 0;
        J_Puv_Pc(1,1) = fy * inv_z;
        J_Puv_Pc(1,2) = -fy * y * inv_z2;

       //p195页 8.14公式
        Matrix<double,3,6> J_Pc_kesi = Matrix<double,3,6>::Zero();
        J_Pc_kesi(0,0) = 1;
        J_Pc_kesi(0,4) = z;
        J_Pc_kesi(0,5) = -y;
        J_Pc_kesi(1,1) = 1;
        J_Pc_kesi(1,3) = -z;
        J_Pc_kesi(1,5) = x;
        J_Pc_kesi(2,2) = 1;
        J_Pc_kesi(2,3) = y;
        J_Pc_kesi(2,4) = -x;

       
        Matrix<double,1,2> J_I_Puv;
        for(int i = -2; i<2; i++)
            for(int j = -2; j<2; j++) {
                int num = 4 * i + j + 10;//0-15
                //像素梯度，即前后做差除以2
                J_I_Puv(0,0) = (GetPixelValue(targetImg,u+i+1,v+j) - GetPixelValue(targetImg,u+i-1,v+j))/2;
                J_I_Puv(0,1) = (GetPixelValue(targetImg,u+i,v+j+1) - GetPixelValue(targetImg,u+i,v+j-1))/2;
                //
                _jacobianOplusXi.block<1,3>(num,0) = -J_I_Puv * J_Puv_Pc * vertexTcw->estimate().rotationMatrix();//地图点J
                _jacobianOplusXj.block<1,6>(num,0) = -J_I_Puv * J_Puv_Pc * J_Pc_kesi;//位姿J
            }
    }

    virtual bool read(istream &in) {}

    virtual bool write(ostream &out) const {}

private:
    cv::Mat targetImg;  // the target image
    float *origColor = nullptr;   // 16 floats, the color of this point
};

// plot the poses and points for you, need pangolin
// 利用pangolin 绘制点和相机姿态
void Draw(const VecSE3 &poses, const VecVec3d &points);

int main(int argc, char **argv) {

    // read poses and points
    VecSE3 poses;
    VecVec3d points;
    ifstream fin(pose_file); //读取位姿

    while (!fin.eof()) {
        double timestamp = 0;
        fin >> timestamp;
        if (timestamp == 0) break;
        double data[7];
        for (auto &d: data) fin >> d;
        //读取四元数和平移
        poses.push_back(Sophus::SE3d(
                Eigen::Quaterniond(data[6], data[3], data[4], data[5]),
                Eigen::Vector3d(data[0], data[1], data[2])
        ));
        if (!fin.good()) break;
    }
    fin.close();


    vector<float *> color;
    fin.open(points_file); //读取点文件
    while (!fin.eof()) {
        double xyz[3] = {0};
        for (int i = 0; i < 3; i++) fin >> xyz[i]; //读取三维坐标
        if (xyz[0] == 0) break;
        points.push_back(Eigen::Vector3d(xyz[0], xyz[1], xyz[2]));
        float *c = new float[16]; //读取颜色小块
        for (int i = 0; i < 16; i++) fin >> c[i];
        color.push_back(c);
        if (fin.good() == false) break;
    }
    fin.close();

    cout << "poses: " << poses.size() << ", points: " << points.size() << endl;

    // Draw(poses, points);
    // read images
    vector<cv::Mat> images;
    boost::format fmt("./%d.png");
    for (int i = 0; i < 7; i++) {
        images.push_back(cv::imread((fmt % i).str(), 0));
    }
    cout << "images: "<<images.size()<<endl;
    // build optimization problem
    // 构造g2o 优化问题
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> DirectBlock;  // 求解的向量是6＊1的
    DirectBlock::LinearSolverType *linearSolver = new g2o::LinearSolverDense<DirectBlock::PoseMatrixType>();
    //DirectBlock *solver_ptr = new DirectBlock(linearSolver);
    DirectBlock* solver_ptr = new DirectBlock ( unique_ptr<DirectBlock::LinearSolverType>(linearSolver) );
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( unique_ptr<DirectBlock>(solver_ptr) );
    //g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    g2o::SparseOptimizer optimizer; //稀疏优化器
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    // TODO add vertices, edges into the graph optimizer
    // START YOUR CODE HERE

    // // 1. 初始化pose 添加第一个点
    // g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
    // pose->setId(0);
    // pose->setEstimate(g2o::SE3Quat());
    // optimizer.addVertex(pose);
    
    // 2. 添加路标节点
    int index = 1;
    for(int i = 0;i<points.size();i++)
    {
        g2o::VertexSBAPointXYZ * vertexPw = new g2o::VertexSBAPointXYZ();
        vertexPw->setEstimate(points[i]);
        vertexPw->setId(i);
        vertexPw->setMarginalized(true);
        optimizer.addVertex(vertexPw);
    }
    // 3.添加位姿节点
    for(int i = 0;i<poses.size();i++)
    {
        VertexSophus* vertexTcw = new VertexSophus();
        vertexTcw->setEstimate(poses[i]);
        vertexTcw->setId(i+points.size());
        optimizer.addVertex(vertexTcw);
    }

    // 4. 添加边
    index = 1;
    for(int i =0;i<poses.size();i++)
    {   

        for(int j= 0;j<points.size();j++)
        {
            EdgeDirectProjection *edge = new EdgeDirectProjection(color[j],images[i]);
            edge->setVertex(0,dynamic_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(j)));// 设置连接的顶点 路标点
            edge->setVertex(1,dynamic_cast<VertexSophus*>(optimizer.vertex(j+points.size())));// 设置连接的顶点 位资点
            edge->setInformation(Matrix<double,16,16>::Identity());//设置协方差矩阵
            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber();
            rk->setDelta(1.0);
            edge->setRobustKernel(rk); // 设置一个鲁棒的核函数
            optimizer.addEdge(edge);
        }
    }
    
    // END YOUR CODE HERE
    

    // perform optimization 执行优化
    optimizer.initializeOptimization(0);
    optimizer.optimize(200);

    // TODO fetch data from the optimizer
    // START YOUR CODE HERE
    for(int c = 0; c < poses.size(); c++)
        for(int p = 0; p < points.size(); p++) {
            Vector3d Pw = dynamic_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(p))->estimate();
            points[p] = Pw;
            Sophus::SE3d Tcw = dynamic_cast<VertexSophus*>(optimizer.vertex(c + points.size()))->estimate();
            poses[c] = Tcw;
        }
    // END YOUR CODE HERE

    // plot the optimized points and poses
    Draw(poses, points);

    // delete color data
    for (auto &c: color) delete[] c;
    return 0;
}

void Draw(const VecSE3 &poses, const VecVec3d &points) {
    if (poses.empty() || points.empty()) {
        cerr << "parameter is empty!" << endl;
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
        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

        // draw poses
        float sz = 0.1;
        int width = 640, height = 480;
        for (auto &Tcw: poses) {
            glPushMatrix();
            Sophus::Matrix4f m = Tcw.inverse().matrix().cast<float>();
            glMultMatrixf((GLfloat *) m.data());
            glColor3f(1, 0, 0);
            glLineWidth(2);
            glBegin(GL_LINES);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
            glEnd();
            glPopMatrix();
        }

        // points
        glPointSize(2);
        glBegin(GL_POINTS);
        for (size_t i = 0; i < points.size(); i++) {
            glColor3f(0.0, points[i][2]/4, 1.0-points[i][2]/4);
            glVertex3d(points[i][0], points[i][1], points[i][2]);
        }
        glEnd();

        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}

