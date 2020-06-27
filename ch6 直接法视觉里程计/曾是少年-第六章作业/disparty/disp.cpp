#include "opencv2/opencv.hpp"
#include <sophus/se3.h>
#include <Eigen/Core>
#include <vector>
#include <string>
#include <boost/format.hpp>
#include <pangolin/pangolin.h>
#include <numeric>

using namespace std;

typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 2, 6> Matrix26d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;

// Camera intrinsics 内参
double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;
// 基线距离
double baseline = 0.573;
// paths
string left_file = "./left.png";
string right_file = "./right.png";
string disparity_file = "./disparity.png";

// 通过插值得到图像在某一点的值
inline float GetPixelValue(const cv::Mat &img, float x, float y);
//单层直接法
VecVector2d DirectPoseEstimationSingleLayer( const cv::Mat &img1, const cv::Mat &img2, const VecVector2d &px_ref, const vector<double> depth_ref, Sophus::SE3 &T21);
//void DirectPoseEstimationSingleLayer( const cv::Mat &img1, const cv::Mat &img2, const VecVector2d &px_ref, const vector<double> depth_ref, Sophus::SE3 &T21 );
//多层直接法
//void DirectPoseEstimationMultiLayer(const cv::Mat &img1,  const cv::Mat &img2, const VecVector2d &px_ref, const vector<double> depth_ref, Sophus::SE3 &T21);
VecVector2d DirectPoseEstimationMultiLayer(const cv::Mat &img1,  const cv::Mat &img2, const VecVector2d &px_ref, const vector<double> depth_ref, Sophus::SE3 &T21);

int main()
{
    cv::Mat left_img = cv::imread(left_file, 0);
    cv::Mat right_img = cv::imread(right_file,0);
    cv::Mat disparity_img = cv::imread(disparity_file, 0);

    // let's randomly pick pixels in the first image and generate some 3d points in the first image's frame
    cv::RNG rng; //随机数生成类
    int nPoints = 1000;
    int boarder = 40;
    VecVector2d pixels_ref;
    vector<double> depth_ref;

    // generate pixels in ref and load depth data
    for (int i = 0; i < nPoints; i++) {
        //使用uniform指定生成类
        int x = rng.uniform(boarder, left_img.cols - boarder);  // don't pick pixels close to boarder
        int y = rng.uniform(boarder, left_img.rows - boarder);  // don't pick pixels close to boarder
        int disparity = disparity_img.at<uchar>(y, x);
        double depth = fx * baseline / disparity; // 根据视差图可以计算深度 you know this is disparity to depth
        depth_ref.push_back(depth);
        pixels_ref.push_back(Eigen::Vector2d(x, y));
    }

    // 利用单层直接法估计
    Sophus::SE3 T_cur_ref;
    cout<<"T_cur_ref"<<T_cur_ref<<endl;
    VecVector2d pixel_right = DirectPoseEstimationSingleLayer(left_img, right_img, pixels_ref, depth_ref, T_cur_ref);
//    VecVector2d pixel_right = DirectPoseEstimationMultiLayer(left_img, right_img, pixels_ref, depth_ref, T_cur_ref);

    //视差计算
    vector<double > disp;
    vector<double > cost;
    for(int i = 0 ;i<pixels_ref.size();i++)
    {
        double dis =  pixels_ref[i].x()-pixel_right[i].x(); //两点之间的水平视差
        disp.push_back(dis);
//        cout<<"dis: "<<dis<<" "<<GetPixelValue(disparity_img,pixels_ref[i].x(),pixels_ref[i].y())<<endl;
        cost.push_back(dis-GetPixelValue(disparity_img,pixels_ref[i].x(),pixels_ref[i].y()));
    }
    //计算视差的平均误差
    double sum = accumulate(cost.begin(),cost.end(),0.0);
    cout<<"平均误差:"<<sum/cost.size()<<endl;

    //
    return 0;
}

// bilinear interpolation
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

//void DirectPoseEstimationSingleLayer(
//        const cv::Mat &img1,
//        const cv::Mat &img2,
//        const VecVector2d &px_ref,
//        const vector<double> depth_ref,
//        Sophus::SE3 &T21
//) {
//
//    // parameters
//    int half_patch_size = 4;
//    int iterations = 100;
//
//    double cost = 0, lastCost = 0;
//    int nGood = 0;  // good projections
//    VecVector2d goodProjection;
//
//    for (int iter = 0; iter < iterations; iter++) {
//        nGood = 0;
//        goodProjection.clear();
//
//        // Define Hessian and bias
//        Matrix6d H = Matrix6d::Zero();  // 6x6 Hessian
//        Vector6d b = Vector6d::Zero();  // 6x1 bias
//
//        for (size_t i = 0; i < px_ref.size(); i++) {
//
//            // compute the projection in the second image
//            // TODO START YOUR CODE HERE
//            float u = 0, v = 0;
//            //前一帧的点
//            Eigen::Vector3d point_ref = depth_ref[i] * Eigen::Vector3d((px_ref[i][0]-cx)/fx,(px_ref[i][1]-cy)/fy,1);
//            //该点在当前帧的为止
//            Eigen::Vector3d point_cur = T21*point_ref;
//            u = fx*point_cur.x()/point_cur.z()+cx;
//            v = fy*point_cur.y()/point_cur.z()+cy;
//
//            nGood++;
//            goodProjection.push_back(Eigen::Vector2d(u, v));
//
//            // and compute error and jacobian
//            for (int x = -half_patch_size; x < half_patch_size; x++)
//                for (int y = -half_patch_size; y < half_patch_size; y++) {
//
//                    double error =0;
//                    error = GetPixelValue(img1,px_ref[i].x()+x,px_ref[i].y()+y)-GetPixelValue(img2,u+x,v+y);
//                    Matrix26d J_pixel_xi;   // pixel to \xi in Lie algebra
//                    Eigen::Vector2d J_img_pixel;    // image gradients
//
//                    // total jacobian
//                    Vector6d J;
//                    Eigen::Matrix<double,2,1> I_p;
//                    Eigen::Matrix<double,6,2> P;
//                    I_p(0) = 0.5*(GetPixelValue(img2,u+x+1,v+y)-GetPixelValue(img2,u+x-1,v+y));
//                    I_p(1) = 0.5*(GetPixelValue(img2,u+x,v+y+1)-GetPixelValue(img2,u+x,v+y-1));
//                    P(0,0) = fx/point_cur.z();
//                    P(1,0) = 0;
//                    P(2,0) = -fx*point_cur.x()/(point_cur.z()*point_cur.z());
//                    P(3,0) = -fx*point_cur.x()*point_cur.y()/(point_cur.z()*point_cur.z());
//                    P(4,0) = fx+fx*(point_cur.x()*point_cur.x())/(point_cur.z()*point_cur.z());
//                    P(5,0) = -fx*point_cur.y()/point_cur.z();
//
//                    P(0,1) = 0;
//                    P(1,1) = fy/point_cur.z();
//                    P(2,1) = -fy*point_cur.y()/(point_cur.z()*point_cur.z());
//                    P(3,1) = -fy-fy*point_cur.y()*point_cur.y()/(point_cur.z()*point_cur.z());
//                    P(4,1) = fy*point_cur.x()*point_cur.y()/(point_cur.z()*point_cur.z());
//                    P(5,1) = fy*point_cur.x()/point_cur.z();
//
//                    J = -P*I_p;
//
//
//                    H += J * J.transpose();
//                    b += -error * J;
//                    cost += error * error;
//                }
//            // END YOUR CODE HERE
//        }
//
//        // solve update and put it into estimation
//        // TODO START YOUR CODE HERE
//        Vector6d update;
//        update = H.ldlt().solve(b);
//        T21 = Sophus::SE3::exp(update) * T21;
//        // END YOUR CODE HERE
//
//        cost /= nGood;
//
//        if (isnan(update[0])) {
//            // sometimes occurred when we have a black or white patch and H is irreversible
//            cout << "update is nan" << endl;
//            break;
//        }
//        if (iter > 0 && cost > lastCost) {
//            cout << "cost increased: " << cost << ", " << lastCost << endl;
//            break;
//        }
//        lastCost = cost;
//        cout << "cost = " << cost << ", good = " << nGood << endl;
//    }
//    cout << "good projection: " << nGood << endl;
//    cout << "T21 = \n" << T21.matrix() << endl;
//
//    // in order to help you debug, we plot the projected pixels here
//    cv::Mat img1_show, img2_show;
//    cv::cvtColor(img1, img1_show, CV_GRAY2BGR);
//    cv::cvtColor(img2, img2_show, CV_GRAY2BGR);
//    for (auto &px: px_ref) {
//        cv::rectangle(img1_show, cv::Point2f(px[0] - 2, px[1] - 2), cv::Point2f(px[0] + 2, px[1] + 2),
//                      cv::Scalar(0, 250, 0));
//    }
//    for (auto &px: goodProjection) {
//        cv::rectangle(img2_show, cv::Point2f(px[0] - 2, px[1] - 2), cv::Point2f(px[0] + 2, px[1] + 2),
//                      cv::Scalar(0, 250, 0));
//    }
//    cv::imshow("reference", img1_show);
//    cv::imshow("current", img2_show);
//    cv::waitKey();
////    return goodProjection;
//}

VecVector2d DirectPoseEstimationSingleLayer(
        const cv::Mat &img1,
        const cv::Mat &img2,
        const VecVector2d &px_ref,
        const vector<double> depth_ref,
        Sophus::SE3 &T21
) {

    // parameters
    int half_patch_size = 4;
    int iterations = 100;

    double cost = 0, lastCost = 0;
    int nGood = 0;  // good projections
    VecVector2d goodProjection;

    for (int iter = 0; iter < iterations; iter++) {
        nGood = 0;
        goodProjection.clear();

        // Define Hessian and bias
        Matrix6d H = Matrix6d::Zero();  // 6x6 Hessian
        Vector6d b = Vector6d::Zero();  // 6x1 bias

        for (size_t i = 0; i < px_ref.size(); i++) {

            // compute the projection in the second image
            // TODO START YOUR CODE HERE
            float u = 0, v = 0;
            //前一帧的点
            Eigen::Vector3d point_ref = depth_ref[i] * Eigen::Vector3d((px_ref[i][0]-cx)/fx,(px_ref[i][1]-cy)/fy,1);
            //该点在当前帧的为止
            Eigen::Vector3d point_cur = T21*point_ref;
            u = fx*point_cur.x()/point_cur.z()+cx;
            v = fy*point_cur.y()/point_cur.z()+cy;

            nGood++;
            goodProjection.push_back(Eigen::Vector2d(u, v));

            // and compute error and jacobian
            for (int x = -half_patch_size; x < half_patch_size; x++)
                for (int y = -half_patch_size; y < half_patch_size; y++) {

                    double error =0;
                    error = GetPixelValue(img1,px_ref[i].x()+x,px_ref[i].y()+y)-GetPixelValue(img2,u+x,v+y);
                    Matrix26d J_pixel_xi;   // pixel to \xi in Lie algebra
                    Eigen::Vector2d J_img_pixel;    // image gradients

                    // total jacobian
                    Vector6d J;
                    Eigen::Matrix<double,2,1> I_p;
                    Eigen::Matrix<double,6,2> P;
                    I_p(0) = 0.5*(GetPixelValue(img2,u+x+1,v+y)-GetPixelValue(img2,u+x-1,v+y));
                    I_p(1) = 0.5*(GetPixelValue(img2,u+x,v+y+1)-GetPixelValue(img2,u+x,v+y-1));
                    P(0,0) = fx/point_cur.z();
                    P(1,0) = 0;
                    P(2,0) = -fx*point_cur.x()/(point_cur.z()*point_cur.z());
                    P(3,0) = -fx*point_cur.x()*point_cur.y()/(point_cur.z()*point_cur.z());
                    P(4,0) = fx+fx*(point_cur.x()*point_cur.x())/(point_cur.z()*point_cur.z());
                    P(5,0) = -fx*point_cur.y()/point_cur.z();

                    P(0,1) = 0;
                    P(1,1) = fy/point_cur.z();
                    P(2,1) = -fy*point_cur.y()/(point_cur.z()*point_cur.z());
                    P(3,1) = -fy-fy*point_cur.y()*point_cur.y()/(point_cur.z()*point_cur.z());
                    P(4,1) = fy*point_cur.x()*point_cur.y()/(point_cur.z()*point_cur.z());
                    P(5,1) = fy*point_cur.x()/point_cur.z();

                    J = -P*I_p;


                    H += J * J.transpose();
                    b += -error * J;
                    cost += error * error;
                }
            // END YOUR CODE HERE
        }

        // solve update and put it into estimation
        // TODO START YOUR CODE HERE
        Vector6d update;
        update = H.ldlt().solve(b);
        T21 = Sophus::SE3::exp(update) * T21;
        // END YOUR CODE HERE

        cost /= nGood;

        if (isnan(update[0])) {
            // sometimes occurred when we have a black or white patch and H is irreversible
            cout << "update is nan" << endl;
            break;
        }
        if (iter > 0 && cost > lastCost) {
            cout << "cost increased: " << cost << ", " << lastCost << endl;
            break;
        }
        lastCost = cost;
        cout << "cost = " << cost << ", good = " << nGood << endl;
    }
    cout << "good projection: " << nGood << endl;
    cout << "T21 = \n" << T21.matrix() << endl;

    // in order to help you debug, we plot the projected pixels here
    cv::Mat img1_show, img2_show;
    cv::cvtColor(img1, img1_show, CV_GRAY2BGR);
    cv::cvtColor(img2, img2_show, CV_GRAY2BGR);
    for (auto &px: px_ref) {
        cv::rectangle(img1_show, cv::Point2f(px[0] - 2, px[1] - 2), cv::Point2f(px[0] + 2, px[1] + 2),
                      cv::Scalar(0, 250, 0));
    }
    for (auto &px: goodProjection) {
        cv::rectangle(img2_show, cv::Point2f(px[0] - 2, px[1] - 2), cv::Point2f(px[0] + 2, px[1] + 2),
                      cv::Scalar(0, 250, 0));
    }
    cv::imshow("reference", img1_show);
    cv::imshow("current", img2_show);
    cv::waitKey();
    return goodProjection;
}

VecVector2d DirectPoseEstimationMultiLayer(
        const cv::Mat &img1,
        const cv::Mat &img2,
        const VecVector2d &px_ref,
        const vector<double> depth_ref,
        Sophus::SE3 &T21
) {

    // parameters
    int pyramids = 4;
    double pyramid_scale = 0.5;
    double scales[] = {1.0, 0.5, 0.25, 0.125};

    // create pyramids
    vector<cv::Mat> pyr1, pyr2; // image pyramids
    // TODO START YOUR CODE HERE
    cv::Mat dst1,dst2;
    for(int i = 0;i<pyramids;i++)
    {
        cv::resize(img1,dst1,cv::Size(),scales[i],scales[i]);
        cv::resize(img2,dst2,cv::Size(),scales[i],scales[i]);
        pyr1.push_back(dst1);
        pyr2.push_back(dst2);
    }
    // END YOUR CODE HERE

    double fxG = fx, fyG = fy, cxG = cx, cyG = cy;  // backup the old values
    VecVector2d disp;
    for (int level = pyramids - 1; level >= 0; level--) {
        VecVector2d px_ref_pyr; // set the keypoints in this pyramid level
        for (auto &px: px_ref) {
            px_ref_pyr.push_back(scales[level] * px);
        }

        // TODO START YOUR CODE HERE
        // scale fx, fy, cx, cy in different pyramid levels
        fx = fxG*scales[level];
        fy = fyG*scales[level];
        cx = cxG*scales[level];
        cy = cyG*scales[level];
        // END YOUR CODE HERE
        disp =  DirectPoseEstimationSingleLayer(pyr1[level], pyr2[level], px_ref_pyr, depth_ref, T21);
    }
    return disp;
}

//void DirectPoseEstimationMultiLayer(
//        const cv::Mat &img1,
//        const cv::Mat &img2,
//        const VecVector2d &px_ref,
//        const vector<double> depth_ref,
//        Sophus::SE3 &T21
//) {
//
//    // parameters
//    int pyramids = 4;
//    double pyramid_scale = 0.5;
//    double scales[] = {1.0, 0.5, 0.25, 0.125};
//
//    // create pyramids
//    vector<cv::Mat> pyr1, pyr2; // image pyramids
//    // TODO START YOUR CODE HERE
//    cv::Mat dst1,dst2;
//    for(int i = 0;i<pyramids;i++)
//    {
//        cv::resize(img1,dst1,cv::Size(),scales[i],scales[i]);
//        cv::resize(img2,dst2,cv::Size(),scales[i],scales[i]);
//        pyr1.push_back(dst1);
//        pyr2.push_back(dst2);
//    }
//    // END YOUR CODE HERE
//
//    double fxG = fx, fyG = fy, cxG = cx, cyG = cy;  // backup the old values
//    for (int level = pyramids - 1; level >= 0; level--) {
//        VecVector2d px_ref_pyr; // set the keypoints in this pyramid level
//        for (auto &px: px_ref) {
//            px_ref_pyr.push_back(scales[level] * px);
//        }
//
//        // TODO START YOUR CODE HERE
//        // scale fx, fy, cx, cy in different pyramid levels
//        fx = fxG*scales[level];
//        fy = fyG*scales[level];
//        cx = cxG*scales[level];
//        cy = cyG*scales[level];
//        // END YOUR CODE HERE
//        DirectPoseEstimationSingleLayer(pyr1[level], pyr2[level], px_ref_pyr, depth_ref, T21);
//    }
//
//}