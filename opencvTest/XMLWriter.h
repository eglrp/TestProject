#pragma once
#include "./TinyXML/tinyxml.h"
#include "./TinyXML/tinystr.h"
#include "SharedMethod.h"

//使用TinyXML
class XMLWriter
{
public:
	XMLWriter(){  }
	///保存单个相机的内参数、畸变参数、误差参数和标定图片外参数
	static void writeCamPara(std::string filename, const CamPara& camparameter);

	///保存相机组姿态参数，保存了Left_Mid和Right_Mid
	//static void writeCamGroupPara(std::string filename, const CamGroupPara& camGroupParameter);

	///保存角点数据，包括图像尺寸、角点二维坐标、三维坐标（z=0)、每针角点数、图像索引
	static void writeCornerData(std::string filename, const CalibrationData& m_cornerData);

	///保存点云数据
	static void writePointsCloud(std::string filename, const std::vector<cv::Point3f>& points);

	///保存光笔测头标定的数据：1、光笔坐标系下的光斑中心点；2、测头标定出的测量坐标系下的测头中心位置
	static void writeLightPenPara(std::string filename, const std::vector<Vec4f>& pnts, const Point3f probeCenter);

	///保存相机和投影机参数，保存了各自的内参数、畸变参数，以及联合标定参数
	static void WriteCamProPara(std::string filename, const CamPara& camParameter, const CamPara& proParameter, const struct RT& camproRT);

	///保存姿态参数，以4乘4矩阵形式保存
	static void WriteRT44(std::string filename,const vector<Mat>& Outputs);

	//**************ZZL add****************//
	//保存立体靶标标定数据
	//static void WriteObjData(std::string filename, const ObjectPara& ObjParam);
	//static void WriteRobotPluse(std::string filename, const vector<vector<double>> plu_pos);

	//************************************庄磊磊***********************************//
	static void WriteStereoPara(std::string filename, const CamPara& camLPara, const CamPara& camRPara, const struct RT& relativeRT,const Size imgSize);
	//保存两点坐标
	static void WritePoints(std::string filename, vector<Point>& points);

private:

	///保存单个相机标定图片的外参数，R以矩阵形式保存
	static void writeImageRT(TiXmlDocument& doc,const CamPara& camparameter);

	///保存一对相机的姿态参数，R以向量形式保存
	static void writeGroupRT(TiXmlElement* element,const double RotVector[3],const double TraVector[3]);
};

