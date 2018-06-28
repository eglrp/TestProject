#include "CrossLaserSensor.h"
#include "CrossLaserSenorAlgorithm.h"
#include "cameraBase.h"


CrossLaserSensor::CrossLaserSensor(abstractCamera *camera)
{
	remodeler = new CrossLaserSenorAlgorithm();
	scannerSet = new laserScannerSetting();
	if ((this->camera = camera) == NULL) {
		err_inf = "camera is null";
	}
}


CrossLaserSensor::~CrossLaserSensor()
{
	delete scannerSet;
	delete camera;
	delete remodeler;
}

bool CrossLaserSensor::Scan(std::vector<cv::Point3f> &pointCloud, int timeout){
	if (!hadInitialied) {
		err_inf = "no initialization";
		return false;
	}
	cv::Mat img;
	if (!camera->captureImage(img)){
		err_inf = camera->getError();
		return false;
	}
	if (!remodeler->remodel(img, pointCloud)){
		err_inf = remodeler->m_errInf;
		return false;
	}
}

bool CrossLaserSensor::initializeDevice(const std::string paraFilePath, LaserDirection laserDir){
	//todo::
	return false;
}

bool CrossLaserSensor::saveSetCfg(const std::string setFilePath){
	//todo::
	return false;
}