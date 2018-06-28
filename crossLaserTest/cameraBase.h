//*************************************************************************
//˵������������Ļ���
//���ߣ�ZZL
//����ʱ�䣺2018/03/30
/*��ע�������޸ĸýӿڣ����������߽���ȷ��,�ýӿ��õ���opencv���������ͣ���Ҫ����
opencv����ʹ��
*/
//*************************************************************************
#ifndef CAMERABASE_H
#define CAMERABASE_H
#include "cv.h"
#include <vector>
using namespace std;

enum triggerMode
{
	SOFT_TRIGGER = 0, //������
	EXT_TRIGGER, //�ⴥ��
	VEIDO //��Ƶ��ģʽ
};

class abstractCamera
{
public:
	abstractCamera() :Connected(false), err_inf("") {};
	virtual ~abstractCamera() {};
	//***************************************************************
	// ����˵��:    �����������
	// �� �� ֵ:    void
	// ��    ��:    @camID output�����ID,����������������ص����ID����string���ͣ���װ��������ֶ�ӳ���string����
	// ��    ע��   
	//***************************************************************
	virtual void searchCamera(vector<string> &camIDs) = 0;
	//***************************************************************
	// ����˵��:    �������
	// �� �� ֵ:    bool
	// ��    ��:    
	// ��    ע��   
	//***************************************************************
	virtual bool connectCamera(string camID) = 0;
	//***************************************************************
	// ����˵��:    �Ͽ�����
	// �� �� ֵ:    bool
	// ��    ��:    
	// ��    ע��   
	//***************************************************************
	virtual bool disconnectCamera() = 0;
	//***************************************************************
	// ����˵��:    ��ȡһ��ͼƬ
	// �� �� ֵ:    bool 
	// ��    ��:    @image output��ȡ��ͼƬ����
	// ��    ע��   
	//***************************************************************
	virtual bool captureImage(cv::Mat &image) = 0;

	//***************************************************************
	// ����˵��:    ��������ع�ʱ��
	// �� �� ֵ:    bool
	// ��    ��:    @time input����ع�ʱ��
	// ��    ע��   ʱ�䵥λ:ms
	//***************************************************************
	virtual bool setExposure(const double time) = 0;

	//***************************************************************
	// ����˵��:    �����������
	// �� �� ֵ:    bool
	// ��    ��:    @scaler input����������С
	// ��    ע��   
	//***************************************************************
	virtual bool setGain(const double scaler) = 0;
	//***************************************************************
	// ����˵��:    ���ô���ģʽ
	// �� �� ֵ:    bool
	// ��    ��:    @scaler input����������С
	// ��    ע��   
	//***************************************************************
	virtual bool setTriggerMode(const triggerMode mode) = 0;

	//***************************************************************
	// ����˵��:    ��ȡ������ع�ʱ��
	// �� �� ֵ:    bool
	// ��    ��:    
	// ��    ע��   
	//***************************************************************
	virtual bool getExposure(double &time) 
	{
		err_inf = "�޴˲���";
		return false;
	};
	//***************************************************************
	// ����˵��:    ��ȡ�������
	// �� �� ֵ:    bool
	// ��    ��:    
	// ��    ע��   
	//***************************************************************
	virtual bool getGain(double &scaler)
	{
		err_inf = "�޴˲���";
		return false;
	}
	
	string getError()
	{
		return err_inf;
	}
	
	bool isConnected(){
		return Connected;
	}
protected:
	string err_inf; //������Ϣ
	bool Connected; //�Ƿ��Ѿ�����
};

#endif