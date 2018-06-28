#pragma once
#include "SharedHead.h"
class cvtool
{
public:
	cvtool();
	~cvtool();
public:
	static void cvtCamPara(CamPara& camParas, Mat& intrinsicPara, Mat& distCoeffs);
};

