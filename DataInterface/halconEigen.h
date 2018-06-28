//*************************************************************************
//说明：halcon和eigen之间数据的转化
//作者：ZZL
//发布时间：2018/02/01
//备注：
//*************************************************************************
#pragma once
#include "datainterface_global.h"
//eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Eigen>

//halcon
#  include "HalconCpp.h"
using namespace HalconCpp;

//将halcon的pose位姿转化成eigen4*4其次矩阵
//note: pose可以是7个元素的或者12个元素
Eigen::Matrix4d halPoseToEigenPose(HTuple pose);

