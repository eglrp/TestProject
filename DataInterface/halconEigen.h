//*************************************************************************
//˵����halcon��eigen֮�����ݵ�ת��
//���ߣ�ZZL
//����ʱ�䣺2018/02/01
//��ע��
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

//��halcon��poseλ��ת����eigen4*4��ξ���
//note: pose������7��Ԫ�صĻ���12��Ԫ��
Eigen::Matrix4d halPoseToEigenPose(HTuple pose);

