//*************************************************************************
//说明：集合了矩阵变换常用的函数
//作者：ZZL
//发布时间：2018/01/22
//备注：代码中引用了opencv3.0 和 eigen 3两个库
//*************************************************************************

//eigen header
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Eigen>

//opencv header
#include "cv.h"
#include"opencv2/opencv.hpp"
#include <opencv2/core/eigen.hpp>


//************************************
// 函数名称：   将四元素转化成3*3旋转矩阵
// 函数说明:    
// 生成日期:    2018/01/22
// 返 回 值:    
// 参    数:    quaternion为指向4个元素的数组的指针，数组元素的顺序为四元素的四个参数：w,x,y,z
// 备    注：   
//************************************
cv::Mat QuaternionToR33(double *quaternion);


//************************************
// 函数说明:    将3*3的旋转矩阵转化成四元素
// 生成日期:    2018/01/22
// 返 回 值:    quaterniond
// 参    数:    rt33 指向9个元素的数组，注：第1~3个元素代表x轴在参考坐标系下的分量，后面以此类推
// 备    注：   
//************************************
Eigen::Quaterniond R33ToQuaternion(double *rt33);
//************************************
// 函数说明:    将3*3的旋转矩阵转化成四元素
// 生成日期:    2018/01/23
// 返 回 值:    quaterniond
// 参    数:    rt33 3*3的矩阵，注：矩阵的第一列代表x轴在参考坐标系下的分量，后面以此类推
// 备    注：   
//************************************
Eigen::Quaterniond R33ToQuaternion(cv::Mat_<double> rt33);

//生成4*4的平移量齐次矩阵
cv::Mat_<double> transl(double x, double y, double z);

//生成绕z轴旋转的齐次矩阵 角度单位：弧度
cv::Mat_<double> trotz(double angle);

//生成绕y轴旋转的齐次矩阵 角度单位：弧度
cv::Mat_<double> troty(double angle);

//生成绕x轴旋转的齐次矩阵 角度单位：弧度
cv::Mat_<double> trotx(double angle);
