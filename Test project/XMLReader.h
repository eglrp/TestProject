#pragma once
#include "SharedMethod.h"
#include "SharedHead.h"
#include "./TinyXML/tinyxml.h"
#include "./TinyXML/tinystr.h"

class XMLReader
{
public:
	XMLReader(){  }

	///读取单个相机的内参数、畸变参数、误差参数、标定板外参数
	static bool readCamPara(std::string filename,CamPara& m_camPara);

	///读取相机组姿态参数，读取了Left_Mid、Right_Mid和Right_Left
	//static bool readCamGroupPara(std::string filename,CamGroupPara& m_camGroupPara);

	///读取角点数据，包括图像尺寸、角点二维坐标、三维坐标（z=0)、每针角点数、图像索引
	static bool readCornerData(std::string filename, CalibrationData& m_cornerData);

	///读取三维点云数据
	static bool readPointsCloud(std::string filename, std::vector<cv::Point3f>& m_points);

	///读取光笔标定的结果
	static bool readLightPenPara(std::string filename,std::vector<TagPoint3f>& pnts,cv::Point3f& pnt);

	///读取相机和投影机参数，包括各自的内参数、畸变参数，以及联合标定参数
	static bool ReadCamProPara(std::string filename, CamPara& camPara, CamPara& proPara, RT& camproRT);

	///读取姿态参数，以4乘4矩阵形式
	static bool ReadRT44(std::string filename,vector<Mat>& Inputs);
	//*************************************zzl add *****************************//

	///读取立体标靶标定参数文件
	static bool readObjPara(std::string filename, ObjectPara& ObjParam);
	static bool readpos61(std::string filename, vector<Vec6f>& pose);

	//************************************庄磊磊***********************************//
	static bool ReadStereoPara(std::string filename, CamPara& camLPara, CamPara& camRPara, struct RT& relativeRT,Size& imgSize);
	//读取二维点文件
	static bool ReadPoints(std::string filename, vector<Point>& points);
	
private:


	///解析内参数元素
	static bool parseIntrinsicparametersElement(TiXmlHandle hDoc, CamPara& m_camPara);

	///解析畸变元素
	static bool parseDistortionsElement(TiXmlHandle hDoc, CamPara& m_camPara);

	///解析误差元素，不参与计算不需要解析
	static bool parseErrorElement(TiXmlHandle hDoc, CamPara& m_camPara);

	///解析图像尺寸元素
	static bool parseImageSizeElement(TiXmlHandle hDoc, CalibrationData& m_cornerData);

	///解析角点二维坐标元素
	static bool parseCornerPointsElement(TiXmlHandle hDoc,const vector<int>& CornerNumPerFrame,CalibrationData& m_cornerData);

	///解析角点三维点（z=0)元素
	static bool parseFeaturePointsElement(TiXmlHandle hDoc,const vector<int>& CornerNumPerFrame, CalibrationData& m_cornerData);

	///解析每针角点数元素
	static bool parseCornerNumPerFrameElement(TiXmlHandle hDoc, vector<int>& CornerNumPerFrame);

	///解析标定图片索引元素
	static bool parseFrameNumListElement(TiXmlHandle hDoc, CalibrationData& m_cornerData);

	//*****************庄磊磊******************//
	///解析左相机内参数元素
	static bool parseCamLIntrinsicparametersElement(TiXmlHandle hDoc, CamPara& m_camPara);
	//解析右相机内参数元素
	static bool parseCamRIntrinsicparametersElement(TiXmlHandle hDoc, CamPara& m_camPara);
	//解析左相机畸变元素
	static bool parseCamLDistortionsElement(TiXmlHandle hDoc, CamPara& m_camPara);
	//解析右相机畸变元素
	static bool parseCamRDistortionsElement(TiXmlHandle hDoc, CamPara& m_camPara);
	//解析RT元素
	static bool parseRTElement(TiXmlHandle hDoc,RT& steroRTPara);
	//解析立体参数中Size元素
	static bool parseStereoParaSize(TiXmlHandle hDoc,Size& imgSize);
	//解析二维点文件
	static bool parsePoints(TiXmlHandle hDoc, vector<Point>& points);
	//************************庄磊磊*****************************//

	///解析Left_Mid元素
	//static bool parseLeftMidElement(TiXmlHandle hDoc, CamGroupPara& m_camGroupPara);

	///解析Right_Mid元素
	//static bool parseRightMidElement(TiXmlHandle hDoc, CamGroupPara& m_camGroupPara);

	///解析Right_Left元素
	//static bool parseRightLeftElement(TiXmlHandle hDoc, CamGroupPara& m_camGroupPara);

	///解析标定板外参数元素
	static bool parseImageRTElement(TiXmlHandle hDoc, CamPara& m_camPara);

	///解析联合标定文件中相机标定元素
	static bool parseIntrinsicCamParametersElement( TiXmlHandle hDoc, CamPara& camPara );

	///解析联合标定文件中相机畸变元素
	static bool parseCamDistortionsElement(TiXmlHandle hDoc, CamPara& camPara);

	///解析联合标定文件中投影机标定元素
	static bool parseIntrinsicProParametersElement( TiXmlHandle hDoc, CamPara& proPara );

	///解析联合标定文件中投影机畸变元素
	static bool parseProDistortionsElement(TiXmlHandle hDoc, CamPara& proPara);

	///解析联合标定外参数元素
	static bool parsesteroRTElement(TiXmlHandle hDoc, RT& steroRTPara);

	//***************ZZL add****************************//
	//解析立体标靶标定特征点尺寸元素
	static bool parseFacePntsElement( TiXmlHandle hDoc, ObjectPara& ObjParam );

	///解析立体标靶面相对位姿关系元素
	static bool parseRT2oneElement( TiXmlHandle hDoc, ObjectPara& ObjParam );

};
