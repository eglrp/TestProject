#include "stdafx.h"
#include "cvtool.h"


cvtool::cvtool()
{
}


cvtool::~cvtool()
{
}

void cvtool::cvtCamPara(CamPara& camParas, Mat& intrinsicPara, Mat& distCoeffs)
{
	intrinsicPara.create(3, 3, CV_64FC1);
	distCoeffs.create(1, 4, CV_64FC1);

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			intrinsicPara.at<double>(i, j) = camParas.CameraIntrinsic[i][j];
		}
	}
	for (int i = 0; i < 4; i++)
	{
		distCoeffs.at<double>(0, i) = camParas.DistortionCoeffs[i];
	}
}
