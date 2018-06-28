#pragma  once
//cam max frame rate
#define MAX_FRAMERATE 150;

//trigger mode
enum TriggerMode_s
{
	TimeMode = 0,
	EncoderMode,
	ExternalInputMode,
	ManualMode,
};
//move dir of movement or rotation
enum MOVEDIR
{
	POSITIVE = 1,
	NAGETIVE = -1,
};

//laser scanner setting
struct laserScannerSetting
{
	TriggerMode_s mode = EncoderMode; //trigger mode	
	double exposureTime = 1; //exposure time,unit: ms
	int recontructionTimeOut = 1000; //recontrucition timeout
	int FrameRate = 140;//frame rate
	//time mode
	double captureTime = 1000 / FrameRate; //trigger duration time, unit:s
	double moveSpeed = 100; //scanner move speed on the y axis
	//encoder params
	int encoderSpace = 350; //trigger pulse step 
	double stepDis = 0.002; //distance per  pulse, unit:mm
	int triggerNum = 1000; //trigger total number
	int startPositon = 60000;//start triggger position	
	MOVEDIR moveDir = POSITIVE; //move dir
};