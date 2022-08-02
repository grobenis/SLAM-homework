/****************************
 * 题目：四元数球面线性插值
 * 我们用智能手机采集了图像序列和IMU数据，由于IMU帧率远大于图像帧率，需要你用Slerp方法进行四元数插值，使得插值后的IMU和图像帧对齐
 * 已知某帧图像的时间戳为：t =700901880170406，离该图像帧最近的前后两个时刻IMU时间戳为：
 * t1 = 700901879318945，t2 = 700901884127851
 * IMU在t1, t2时刻测量得的旋转四元数为：
 * q1x=0.509339, q1y=0.019188, q1z=0.049596, q1w=0.858921；
 * q2x=0.509443, q2y=0.018806, q2z=0.048944,q2w=0.858905
 * 根据上述信息求IMU对齐到图像帧的插值后的四元数
 * 本程序学习目标：
 * 学习四元数球面线性插值方法，注意实际使用中需要考虑的问题。
 *
 * 作者：公众号：计算机视觉life。发布于公众号旗下知识星球：从零开始学习SLAM
 * 时间：2019.08
 ****************************/
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;
using namespace Eigen;

// 四元数球面线性插值简化方法：v'=v1*cosθ' + v⊥*sinθ'，原理见公众号推送文章
Quaterniond slerp(double t, Quaterniond &q1, Quaterniond &q2)
{
	// ---- 开始你的代码 ----- //
	// 参考公众号推送文章
	Quaterniond deltaQ = q1.inverse() * q2;
	// Eigen::AngleAxisd rotation_vector(deltaQ);
	// double theta = rotation_vector.angle();

	double theta = acos(q1.dot(q2));
	// cout << rotation_vector.angle() << endl;
	// cout << rotation_vector.matrix() << endl;
	double q_slerp_w, q_slerp_x, q_slerp_y, q_slerp_z;
	if (theta < 0.001)
	{
		q_slerp_w = (1 - t) * q1.w() + t * q2.w();
		q_slerp_x = (1 - t) * q1.x() + t * q2.x();
		q_slerp_y = (1 - t) * q1.y() + t * q2.y();
		q_slerp_z = (1 - t) * q1.z() + t * q2.z();
	}
	else
	{
		q_slerp_w = sin((1 - t) * theta) / sin(theta) * q1.w() + sin(t * theta) / sin(theta) * q2.w();
		q_slerp_x = sin((1 - t) * theta) / sin(theta) * q1.x() + sin(t * theta) / sin(theta) * q2.x();
		q_slerp_y = sin((1 - t) * theta) / sin(theta) * q1.y() + sin(t * theta) / sin(theta) * q2.y();
		q_slerp_z = sin((1 - t) * theta) / sin(theta) * q1.z() + sin(t * theta) / sin(theta) * q2.z();
	}
	Quaterniond q_slerp(q_slerp_w, q_slerp_x, q_slerp_y, q_slerp_z);
	if (theta < 0.001)
	{
		q_slerp = q_slerp.Identity();
	}
	// ---- 结束你的代码 ----- //
	return q_slerp;
}
int main(int argc, char **argv)
{
	double t_img(700901880170406), t1_imu(700901879318945), t2_imu(700901884127851);
	Quaterniond q1 = Quaterniond(0.858921, 0.509339, 0.019188, 0.049596);
	Quaterniond q2 = Quaterniond(0.858905, 0.509443, 0.018806, 0.048944);
	double t = (t_img - t1_imu) / (t2_imu - t1_imu);
	Quaterniond q_slerp = slerp(t, q1, q2);
	cout << "插值后的四元数：q_slerp =\n"
		 << q_slerp.coeffs() << endl; // coeffs的顺序是(x,y,z,w)

	return 0;
}
