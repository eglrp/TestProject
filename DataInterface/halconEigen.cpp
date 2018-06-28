#include "halconEigen.h"

Eigen::Matrix4d halPoseToEigenPose(HTuple pose)
{
	Eigen::Matrix4d _ma4 = Eigen::Matrix4d::Identity();
	HTuple _pose;
	if (pose.Length() == 7)
	{
		PoseToHomMat3d(pose, &_pose);
	}

	if (pose.Length() == 12)
	{
		_pose = pose;
	}
	_ma4 << _pose[0], _pose[1], _pose[2], _pose[3],
		_pose[4], _pose[5], _pose[6], _pose[7],
		_pose[8], _pose[9], _pose[10], _pose[11],
		0, 0, 0, 1;
	return _ma4;
}

