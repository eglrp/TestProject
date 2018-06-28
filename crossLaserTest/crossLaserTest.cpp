// crossLaserTest.cpp : �������̨Ӧ�ó������ڵ㡣
//

#include "Algorithm.h"
#include "stdafx.h"
#include "XMLReader.h"
#include "XMLWriter.h"
#include "laserSensorAlgorithm.h"
#include <memory>
#include "test.h"

#include <boost/shared_ptr.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/property_tree/json_parser.hpp>


using namespace cv;

Mat org, dst, img, tmp;
Rect roi;

void StegerLine(Mat src, vector <double> &Pt);
void classPt(Mat src, vector <CAlgorithm::laserCenter> LaserCenters, vector <cv::Point2f> &class1, vector <cv::Point2f> &class2);
void on_mouse(int event, int x, int y, int flags, void *ustc);
Mat thinImage(const cv::Mat & src, const int maxIterations = -1);
Mat getLaserImg(const cv::Size &imgsize, const int laserWidth, vector<int> &center_col, const int stripeType);
Mat getHmat(string imagePath, const double cornerdis, const int cornerType = 2, const Point2i cornerCount = Point2i(7, 7));

enum class COLOR{
	red,
	black
};

class B{
public:
	~B(){ cout << "dispose B"<<endl; };
	int zn = 9;
	typedef  std::vector<int> kk;
	enum class TE{
		TE2,
		TE3
	};
};
class A{
public:
	~A(){ cout << "dispose A" << endl; };
	A(){ b = new B; };
	static const int num = 5;
	int _name = 9;
	int po[4];
	static const vector<int> df;
	B::kk dg;
	B *b;
};
const vector<int> A::df = { 1, 2, 4 };

int _tmain(int argc, _TCHAR* argv[])
{
#if 0
	std::tuple<int, double, char> tp = make_tuple(30,3.45,'j');
	auto id = std::get<0>(tp);

	ifstream fi("mark.txt");
	string str;
	double r, l;
	while (fi >>str >> r >> l)
	{
		cout << str << " "<< r << " " << l << endl;
	}
	cout << "read end" <<endl;
#endif


	//shared_ptr����
#if 0
	int i= 0;

	for (int i = 0; i < 4; i++)
	{
		shared_ptr <A> a(new A);
	}

	shared_ptr<test> st = test::init();
	st->memberd = 12;

	shared_ptr<test> st1 = test::init();
	cout << st1->memberd << endl;
	st1->memberd = 15;
	cout << st->memberd << endl;
#endif

	//jeson �ļ�����
	//jeson��֧������
#if 0
	using ptree = boost::property_tree::ptree;
	ptree m_tree;
	boost::property_tree::read_json("./config.json", m_tree);
	string port = m_tree.get<string>("grasper.port");
	double grasper_gap = m_tree.get<double>("grasper.grasper_gap");
	Sleep(3);

	// write jeson
	ptree _root;
	_root.put("name", "zheng zelong");
	_root.put("sex", "man");
	ptree score;
	score.put("china", "86");
	score.put("computor", "100");
	score.put("english", "86");
	_root.add_child("score", score);
	stringstream str;
	str << 25 << " " << 36 << " " << 63 << " " << 53;
	_root.put("array", str.str());
	boost::property_tree::write_json("out.json", _root);


	ptree ptree1;
	boost::property_tree::read_json("out.json", ptree1);
	ptree chrs = ptree1.get_child("score");
	
	string d = ptree1.get<string>("array");
	cout << d << endl;
	//another write method
	//std::string str_json = "{\"name\":\"zheng zelong\",\"�Ա�\":\"��\",\"�ɼ�\":{\"����\":\"63\",}}";
	//stringstream s;

#endif
	
#if 0
	
#endif

	//����boost������ϵ�л�
#if 0
	//���л�����:û����
	/*
	int num[5] = { 2, 34 };
	ofstream osm("./config.cfg", ios::out | ios::binary);
	boost::archive::text_oarchive ar(osm);
	ar & num;
	osm.close();

	ifstream ism("./config.cfg", ios::in | ios::binary);
	boost::archive::text_iarchive ir(ism);
	int rd[5];
	ir & rd;
	ism.close();
	cout << rd[0] << endl;
	*/


	//���л�mat��û����
	/*
	Mat in = Mat::eye(4, 4, CV_64FC1);
	ofstream osm("./config.cfg", ios::out | ios::binary);
	boost::archive::text_oarchive ar(osm);
	cvtool::cvmatSerializeA(ar,in,1);
	osm.close();

	ifstream ism("./config.cfg", ios::in | ios::binary);
	boost::archive::text_iarchive ir(ism);
	Mat out;
	cvtool::cvmatSerializeA(ir, out, 1);
	cout << out << endl;
	osm.close();
	*/

	//���л��Զ�������:û����
	laserSensor::SensorPara para;
	para.ratio = 10;
	ofstream osm("./sensorPara.par", ios::out);
	boost::archive::text_oarchive ar(osm);
	para.serializeA(ar, 1);
	osm.close();

	ifstream ism("./sensorPara.par", ios::in);
	boost::archive::text_iarchive ir(ism);
	laserSensor::SensorPara outp;
	outp.serializeA(ir,1);
	cout << outp.ratio << endl;
	ism.close();
	

#endif

	//c++�Դ����л��ĵ����ԣ�������ջ�ϵı���û���⣬�������ڶ��ϵı���������ڴ汨��
#if 0
	A para;
	para._name = 15;
	ofstream osm("./config.cfg", ios::out | ios::binary);
	osm.write((char*)&para, sizeof(A));
	osm.close();

	A pd;
	ifstream ism("./config.cfg", ios::in | ios::binary);
	ism.read((char *)&pd, sizeof(A));
	cout << para._name << endl;
	ism.close();

	//laserSensor::SensorPara para;
	//para.ratio = 100;
	//ofstream osm("./config.cfg", ios::out | ios::binary);
	//osm.write((char*)&para, sizeof(laserSensor::SensorPara));
	//osm.close();

	//laserSensor::SensorPara pd;
	//ifstream ism("./config.cfg", ios::in | ios::binary);
	//ism.read((char *)&pd, sizeof(laserSensor::SensorPara));
	//cout << para.ratio << endl;
	//ism.close();
#endif

	//list test
#if 0
	std::list<int> _list;
	_list.push_back(1);
	_list.push_back(1);
	_list.push_back(1);
	_list.push_back(2);
	_list.push_back(3);
	_list.push_back(1);


	auto itr = _list.begin();
	while (itr != _list.end())
	{
		if (*itr == 1)
			itr = _list.erase(itr);
		else
			itr++;
	}
	vector<int> ptm(100, 1);
	std::copy(ptm.begin(), ptm.end(), back_inserter(_list));
	cout << _list.size()<<endl;
#endif


	//*****************************************************************
	//��֤����궨
#if 1
	/*
	laserSensor laser;
	Mat refImg = imread("./clipimage/laser1.bmp");
	Mat othImg = imread("./clipimage/laser2.bmp");
	double ratio = laser.calibrateZtoLaser(refImg, othImg, 1);
	Mat testImg = imread("./clipimage/laser3.bmp");
	cout << laser.getZ(testImg) << endl;
	*/
	laserSensorAlgorithm laser;
	/*
	vector <double> z;
	vector<Mat> Imgs;
	for (int i = 1; i < 5; i++)
	{
	stringstream str;
	str << "./clipimage/laser/laser"<< i << ".bmp";
	Imgs.push_back(imread(str.str()));
	z.push_back(1 * (i - 1));
	}
	if (laser.calibrateZtoLaser(Imgs, z)){
	//Mat refImg = imread("./clipimage/refImg.bmp");
	//laser.setLaserRefPlane(refImg);
	laser.writeParameter("./sensorPara.par");
	}
	*/
	
	laser.readParameter("sensorPara.par");
	string  str = "./clipimage/laser/laserTest5.bmp";
	Mat tesimg = imread(str);
	double he = laser.getZ(tesimg);
	cout << "height 5 = " << -he + 3.85 << endl;

	str = "./clipimage/laser/laserTest6.bmp";
	tesimg = imread(str);
	he = laser.getZ(tesimg);
	cout << "height 6 = " << -he + 3.85 << endl;

	str = "./clipimage/laser/laserTest9.bmp";
	tesimg = imread(str);
	he = laser.getZ(tesimg);
	cout << "height 9 = " << -he + 3.85 << endl;
	
#endif

	//*****************************************************************
	//��֤��Ӧ�Ծ���궨
	//��֤�������Ա仯�������ת�����ǵ�λ�󣬼�������ʵ��������õĳߴ����0.05mm���ҡ�
#if 0
	vector<Mat> hmats;
	vector <double> z;
	for (int i = 1; i < 4; i++)
	{
		stringstream str;
		str << "./clipimage/Hmat/" << i << ".bmp";
		Mat hmat = getHmat(str.str(), 0.8, 2, Point2i(8, 8));
		cout << hmat << endl <<endl;
		hmats.push_back(hmat);
		z.push_back((i-1)*1);
	}
	laserSensorAlgorithm laser;
	laser.readParameter("./sensorPara.par");
	double ratio = 0; Point2d dir;
	laser.ratioxyFromZ(hmats, z, ratio, dir);
	laser.writeParameter("./sensorPara.par");
	
	//��֤
	Mat tr = getHmat("./clipimage/Hmat/2.bmp",0.8,2,Point2f(8,8));
	Mat te = laser.getHmat(1);
	cout << tr << endl;
	cout << te << endl;
#endif

	//*****************************************************************
	//����豸λ�ù�ϵ�궨
#if 0
	//��ȡ��������ϵ��ִ�л�������ϵ�µ�λ��
	float pulseNumPerMM = 2000.0;
	vector<cv::Point3f> corIndevice;
	corIndevice.push_back(Point3f(396810 / pulseNumPerMM, 365795 / pulseNumPerMM, 0)); // ԭ��
	corIndevice.push_back(Point3f(396810 / pulseNumPerMM, 352895 / pulseNumPerMM, 0)); //x���ϵ�һ�㣨�궨������ϵ��x�ᣬͼ����з���Ϊx�ᣩ
	corIndevice.push_back(Point3f(408810 / pulseNumPerMM, 365795 / pulseNumPerMM, 0)); //y���ϵ�һ��
	Mat_<double> wordInBase;
	CAlgorithm::frameFrom3Pnts(corIndevice, wordInBase);
	//��Ϊtcp����ϵ��baseֻ��ƽ�ƣ�����ʱ����z����
	wordInBase(0, 3) -= 660005 / pulseNumPerMM;
	wordInBase(1, 3) -= 249999 / pulseNumPerMM;
	wordInBase(2, 3) -= 0;
	laserSensorAlgorithm _laser;
	_laser.readParameter("./sensorPara.par");
	_laser.m_para.worldInHand = wordInBase;
	_laser.writeParameter("./sensorPara.par");
#endif

	//��֤
#if 0
	laserSensorAlgorithm _laser;
	_laser.readParameter("./sensorPara.par");

	double imshowScaler = 0.5;
	cv::namedWindow("img", WINDOW_AUTOSIZE & WINDOW_KEEPRATIO);
	setMouseCallback("img", on_mouse, &imshowScaler);
	org = imread("./clipimage/Hmat/test1.bmp");
	cv::imshow("img", org);
	cv::resizeWindow("img", org.cols*imshowScaler, org.rows *imshowScaler);
	waitKey(0);

	//��ȡroi����Ľǵ�
	Size patternsize(8,8);
	vector<Point2f> corners;
	cvtColor(dst, dst, CV_RGB2GRAY);
	cv::GaussianBlur(dst, dst, Size(5, 5), 0, 0);


	bool patternfound = false;
	//Բ�ǵ���ȡ
	patternfound = cv::findChessboardCorners(dst, patternsize, corners,
		CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE);
	if (patternfound){
		cornerSubPix(dst, corners, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 50, 0.1));
	}
	//���̸�ǵ���ȡ

	for (int i = 0; i < corners.size(); i++){
		corners[i] += Point2f(roi.tl().x, roi.tl().y);
		Point2f tcp(231.3, 187.8);
		Point2f pt = _laser.getXY(corners[i], tcp, 3.5);
		Point2f pt1 = _laser.getXY(corners[i], tcp);
		cout << "corner_" << i << endl;
		std::cout << "pt = " << pt << endl;
		std::cout << "pt1 = " << pt1 << endl;
		cout << endl;
	}


	//laserSensorAlgorithm _laser;
	//_laser.readParameter("./sensorPara.par");
	//while (true){
	//	cout << "����ͼ������꣺(x y)";
	//	float x, y;
	//	cin >> x >> y;
	//	cout << "�������λ�õ����꣺(fx fy)";
	//	int fx, fy;
	//	cin >> fx >> fy;
	//	Mat testimg = imread("./clipimage/test1.bmp");
	//	double z = _laser.getZ(testimg);
	//	Point2f pt = _laser.getXY(Point2f(x, y), Point2f(fx, fy), z);
	//	Point2f pt1 = _laser.getXY(Point2f(x, y), Point2f(fx, fy));
	//	cout << "оƬ�ڻ�������ϵ�µ�λ�ã�" << pt << endl;
	//	cout << "�����Ǹ߶ȱ仯�Ľ����" << pt1 << endl;
	//}

#endif



	//*****************************************************************
	//��֤��Ӧ�Ծ��������ӦZ����ı仯
#if 0
	vector<Mat> hmats;
	for (int i = 1; i < 3; i++){
		double imshowScaler = 0.5;
		cv::namedWindow("img", WINDOW_AUTOSIZE & WINDOW_KEEPRATIO);
		//��ȡROI
		stringstream str; str << "./clipimage/" << i << ".bmp";
		org = imread(str.str());
		setMouseCallback("img", on_mouse, &imshowScaler);
		cv::imshow("img", org);
		cv::resizeWindow("img", org.cols*imshowScaler, org.rows *imshowScaler);
		waitKey(0);

		//��ȡroi����Ľǵ�
		Size patternsize(7, 7);
		vector<Point2f> corners;
		cvtColor(dst, dst, CV_RGB2GRAY);
		cv::GaussianBlur(dst, dst, Size(5, 5), 0, 0);

		//Բ�ǵ���ȡ
		//SimpleBlobDetector::Params params;
		//params.maxArea = 10e5;
		//params.minArea = 100;
		//Ptr<FeatureDetector> blobDetector = SimpleBlobDetector::create(params);
		//bool patternfound = findCirclesGrid(dst, patternsize, corners, CALIB_CB_SYMMETRIC_GRID, blobDetector);
		//���̸�ǵ���ȡ
		
		bool patternfound = cv::findChessboardCorners(dst, patternsize, corners,
		CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE);
		if (patternfound){
		cornerSubPix(dst, corners, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 50, 0.1));
		} //���Ǳ���
		
		for (int i = 0; i < corners.size(); i++){
			corners[i] += Point2f(roi.tl().x, roi.tl().y);
		}
		drawChessboardCorners(org, patternsize, Mat(corners), patternfound);

		//��ʾ��ȡ���
		//cv::imshow("img", org);
		//cv::resizeWindow("img", org.cols * imshowScaler, org.rows * imshowScaler);
		//cv::waitKey(0);

		//��������ϵ�µ�����ֵ
		float step = 12.5;
		vector<Point2f> corInWord;
		int _num = patternsize.width * patternsize.height;
		for (int i = 0; i < 49; i++){
			corInWord.push_back(Point2f(step*(i % patternsize.width), step*(i / patternsize.width)));
		}

		//���㵥Ӧ����(��src��dst�µı任����)
		Mat hmat = findHomography(corners, corInWord);
		hmats.push_back(hmat);
	}
	invert(hmats[0], hmats[0]);
	cout << hmats[0] * hmats[1] << endl; //����ӽ�Ϊֻ��ƽ�����ľ��󣬵�ƽ������ʵ�������ƽ����������Ӧ��
#endif

	//******************************************************************
	//оƬʰȡ
#if 0
	//����һ��������궨
#if 1
	//��ʾ
	double imshowScaler = 0.5;
	cv::namedWindow("img", WINDOW_AUTOSIZE & WINDOW_KEEPRATIO);
	//��ȡROI
	org = imread("./clipimage/1.bmp");
	setMouseCallback("img", on_mouse, &imshowScaler);
	cv::imshow("img", org);
	cv::resizeWindow("img", org.cols*imshowScaler, org.rows *imshowScaler);
	waitKey(0);

	//��ȡroi����Ľǵ�
	Size patternsize(7, 7);
	vector<Point2f> corners;
	cvtColor(dst, dst, CV_RGB2GRAY);
	cv::GaussianBlur(dst, dst, Size(5, 5), 0, 0);

	//Բ�ǵ���ȡ
	SimpleBlobDetector::Params params;
	params.maxArea = 10e5;
	params.minArea = 100;
	Ptr<FeatureDetector> blobDetector = SimpleBlobDetector::create(params);
	bool patternfound = findCirclesGrid(dst, patternsize, corners, CALIB_CB_SYMMETRIC_GRID, blobDetector);
	//���̸�ǵ���ȡ
	/*
	bool patternfound = cv::findChessboardCorners(dst, patternsize, corners,
		CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE);
	if (patternfound){
		cornerSubPix(dst, corners, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 50, 0.1));
	} //���Ǳ���
	*/
	for (int i = 0; i < corners.size(); i++){
		corners[i] += Point2f(roi.tl().x, roi.tl().y);
	}
	drawChessboardCorners(org, patternsize, Mat(corners), patternfound);

	//��ʾ��ȡ���
	//cv::imshow("img", org);
	//cv::resizeWindow("img", org.cols * imshowScaler, org.rows * imshowScaler);
	//cv::waitKey(0);

	//��������ϵ�µ�����ֵ
	int step = 1;
	vector<Point2f> corInWord;
	int _num = patternsize.width * patternsize.height;
	for (int i = 0; i < 49; i++){
		corInWord.push_back(Point2f(step*(i % patternsize.width), step*(i/patternsize.width)));
	}

	//���㵥Ӧ����(��src��dst�µı任����)
	Mat hmat = findHomography(corners, corInWord);
	XMLWriter::WriteRT33("./clipimage/hmat.xml", vector<Mat>(1, hmat));
	//���ο���ƽ�Ƶ�ͼ�������
	//Mat_<double> centerInWord = hmat * (cv::Mat_<double>(3, 1) << org.rows / 2.0, org.cols / 2.0, 1);

	//��ȡ��������ϵ��ִ�л�������ϵ�µ�λ��
	vector<cv::Point3f> corInHand;
	corInHand.push_back(Point3f(101440, 210378, 0));
	corInHand.push_back(Point3f(101440, 204850, 0));
	corInHand.push_back(Point3f(107116, 210385, 0));
	Mat_<double> wordInHand;
	CAlgorithm::frameFrom3Pnts(corInHand, wordInHand);
	wordInHand(0, 3) -= 400000;
	wordInHand(1, 3) -= 105000;
	//wordInHand(0, 3) += centerInWord(0, 0)*1000;
	//wordInHand(1, 3) += centerInWord(1, 0)*1000;
	XMLWriter::WriteRT44("./clipimage/wordinhand.xml", vector<Mat>(1, wordInHand));
#endif
	//ͼ������ϵ->ִ�л�������ϵ
	//test
	//Mat imgp = (cv::Mat_<double>(3, 2) << 0, 90, 
	//									0,90,
	//									1,1);
	//Mat_<double> _w = hmat*imgp;
	//cout << _w << endl;

#if 0
	org = imread("./clipimage/1.bmp");
	vector<Mat> hmat;
	XMLReader::ReadRT33("./clipimage/hmat.xml", hmat);
	vector <Mat> wordInHand;
	XMLReader::ReadRT44("./clipimage/wordinhand.xml", wordInHand);

	while (true){
		cout << "����ͼ������꣺(x y)";
		float x, y;
		cin >> x >> y;
		//x -= org.rows / 2.0;
		//y -= org.cols / 2.0;
		cout << "�������λ�õ����꣺(fx fy)";
		int fx, fy;
		cin >> fx >> fy;
		Mat imgp = (cv::Mat_<double>(3, 1) <<x, y, 1);
		Mat_<double> _w = hmat[0] * imgp;
		//Mat wodp = (cv::Mat_<double>(4, 1) << (_w(0, 0) - centerInWord(0,0)) * 1000,( _w(1, 0) - center(1,0)) * 1000, 0, 1);
		Mat wodp = (cv::Mat_<double>(4, 1) << (_w(0, 0)) * 1000,( _w(1, 0)) * 1000, 0, 1);
		Mat_<double> exp = wordInHand[0] * wodp;
		exp(0, 0) += fx;
		exp(1, 0) += fy;
		cout << "оƬ�ڻ�������ϵ�µ�λ�ã�" << exp.t() << endl;
	}
#endif
#endif

	//******************************************************************
	//ʮ�ּ���������ȡ����
	/*
	for (int i = 15; i < 25; i++)
	{
		stringstream imgPath;
		imgPath << "./+laser test image/" << i << ".bmp";
		Mat img = cv::imread(imgPath.str(), cv::IMREAD_GRAYSCALE);
		//cout << img.at<uchar>(1, 1270) << endl;
		//cout << img.rows <<" "<< img.cols << endl;
		vector <CAlgorithm::laserCenter> laserCenters;
		laserCenters.reserve(1300);
		double _mean, _std;
		Mat gausk;
		cv::Rect ro(0, 0, img.cols, img.rows);
		CAlgorithm::GaussKNew(gausk, 5, 5, 1);
		clock_t t1 = clock();
		//����һ���б��� + ���ķ��� Ч�����ã��ر��ǽ��㴦��Ч���ܲ�
#if 0
		CAlgorithm::CrossLaserCenterDetect(img, 70, 50, 0, laserCenters, _mean, _std, gausk, 0, ro);
		cout << clock() - t1 << endl;
		Mat img_show = cv::imread(imgPath.str(), cv::IMREAD_COLOR);
		//�����ĵ���з���
		vector <Point2f> pnts1,pnts2;
		CAlgorithm::classCrossPt(img, laserCenters, pnts1,pnts2);
		////��ʾ��ȡЧ��
		for (int i = 0; i < pnts1.size(); i++)
		{
			img_show.at<cv::Vec3b>((int)pnts1[i].x, (int)pnts1[i].y) = cv::Vec3b(0, 0, 255);
		}
		for (int i = 0; i < pnts2.size(); i++)
		{
			img_show.at<cv::Vec3b>((int)pnts2[i].x, (int)pnts2[i].y) = cv::Vec3b(0, 255, 0);
		}

		////test ���
		//vector<Point2f>::iterator itr = pts2.begin();
		//vector<Point2f> srcpnts;
		//for (int i = 301; i < 346; i += 4)
		//{
		//	if (i > 311 && i < 334) continue;
		//	srcpnts.push_back(pts2[i]);
		//}
		//vector <Point2f> smoothPnts;
		//CAlgorithm::smoothPoints(srcpnts,smoothPnts,2,1);
		//for (int i = 0; i < smoothPnts.size(); i++)
		//{
		//	img_show.at<cv::Vec3b>((int)smoothPnts[i].x, (int)smoothPnts[i].y) = cv::Vec3b(255, 0, 0);
		//}

#endif

		//��������STEGER������Ч�������˵�ȽϺã�����ĳЩͼƬ����������ȡЧ�����ǲ��á����⣬�ٶ�Ҳ�Ƚ���,���һ�����������ĵ����
#if 0
		vector<double> Pt;
		Pt.reserve(2000);
		StegerLine(img, Pt);
		cout << clock() - t1 << endl;
		Mat img_show = cv::imread(imgPath.str(), cv::IMREAD_COLOR);
		for (int k = 0; k < Pt.size() / 2; k++)
		{
			Point rpt;
			rpt.x = Pt[2 * k + 0];
			rpt.y = Pt[2 * k + 1];
			img_show.at<cv::Vec3b>(rpt.y, rpt.x) = cv::Vec3b(0, 0, 255);
		}
#endif

		//test:����steger�㷨��������ȡ�ֱ���: �ֱ���ȷʵ���ܵ�Ӱ�죬���evernote
#if 0
		vector<int> center_col(1024, 500);
		center_col[50] = 501;
		vector<double> Pt;
		Mat srcimg = getLaserImg(cv::Size(1280, 1024), 7, center_col, 1);
		StegerLine(srcimg, Pt);

		Mat img_show;
		cvtColor(srcimg, img_show, CV_GRAY2RGB);
		for (int k = 0; k < Pt.size() / 2; k++)
		{
			Point rpt;
			rpt.x = Pt[2 * k + 0];
			rpt.y = Pt[2 * k + 1];
			img_show.at<cv::Vec3b>(rpt.y, rpt.x) = cv::Vec3b(0, 0, 255);
		}

#endif
		//�����������෽��
#if 0
		vector <Point2f> pnts1, pnts2;
		CAlgorithm::CrossLaserCenterDetect(img, 70, 20, pnts1, pnts2, ro);
		cout << clock() - t1 << endl;
		Mat img_show = cv::imread(imgPath.str(), cv::IMREAD_COLOR);
		
#endif

		//��ʾ��ȡ��Ч��
		cv::namedWindow("img");
		cv::imshow("img", img_show);
		cv::waitKey(0);
	}
	
	*/
	return 0;
}




/** \brief steger�㷨��ȡ��������
  * \param[in] 
  * \param[out] 
  * \note  
  */
void StegerLine(Mat src, vector <double> &Pt)
{
	Mat img;
	//cvtColor(src, img, CV_BGR2GRAY);
	img = src.clone();

	//test
	Mat show(img.clone());
	cvtColor(show, show, CV_GRAY2BGR);

	//��˹�˲�
	img.convertTo(img, CV_32FC1);
	//GaussianBlur(img, img, Size(0, 0), 6, 6); //�˴��˴�С���ó�0����ʾ�˴�С��sigmaֵ������
	//GaussianBlur(img, img, Size(0, 1), 6, 0); //�˴��˴�С���ó�0����ʾ�˴�С��sigmaֵ������
	clock_t t1 = clock();
	LARGE_INTEGER nFreq;
	LARGE_INTEGER nBeginTime;
	LARGE_INTEGER nEndTime;
	double time;
	QueryPerformanceFrequency(&nFreq);
	QueryPerformanceCounter(&nBeginTime);
	GaussianBlur(img, img, Size(15, 15), 0, 0); //15*15�ĸ�˹�˺�ʱ��5ms����
	QueryPerformanceCounter(&nEndTime);
	time = (double)(nEndTime.QuadPart - nBeginTime.QuadPart) / (double)nFreq.QuadPart;
	cout<<"��˹�˲���ʱ��"<< time<< endl;

	//һ��ƫ����
	Mat m1, m2;
	m1 = (Mat_<float>(1, 2) << 1, -1);  //xƫ��
	m2 = (Mat_<float>(2, 1) << 1, -1);  //yƫ��

	Mat dx, dy;
	filter2D(img, dx, CV_32FC1, m1);
	filter2D(img, dy, CV_32FC1, m2);

	//����ƫ����
	Mat m3, m4, m5;
	m3 = (Mat_<float>(1, 3) << 1, -2, 1);   //����xƫ��
	m4 = (Mat_<float>(3, 1) << 1, -2, 1);   //����yƫ��
	m5 = (Mat_<float>(2, 2) << 1, -1, -1, 1);   //����xyƫ��

	Mat dxx, dyy, dxy;
	filter2D(img, dxx, CV_32FC1, m3);
	filter2D(img, dyy, CV_32FC1, m4);
	filter2D(img, dxy, CV_32FC1, m5);

	//hessian����
	double maxD = -1;
	int imgcol = img.cols;
	int imgrow = img.rows;
	for (int i = 0; i < imgcol; i++)
	{
		for (int j = 0; j<imgrow; j++)
		{
			if (src.at<uchar>(j, i)>200)
			{
				Mat hessian(2, 2, CV_32FC1);
				hessian.at<float>(0, 0) = dxx.at<float>(j, i);
				hessian.at<float>(0, 1) = dxy.at<float>(j, i);
				hessian.at<float>(1, 0) = dxy.at<float>(j, i);
				hessian.at<float>(1, 1) = dyy.at<float>(j, i);

				Mat eValue;
				Mat eVectors;
				eigen(hessian, eValue, eVectors);

				double nx, ny;
				double fmaxD = 0;
				if (fabs(eValue.at<float>(0, 0)) >= fabs(eValue.at<float>(1, 0)))  //������ֵ���ʱ��Ӧ����������
				{
					nx = eVectors.at<float>(0, 0);
					ny = eVectors.at<float>(0, 1);
					fmaxD = eValue.at<float>(0, 0);
				}
				else
				{
					nx = eVectors.at<float>(1, 0);
					ny = eVectors.at<float>(1, 1);
					fmaxD = eValue.at<float>(1, 0);
				}

				double t = -(nx*dx.at<float>(j, i) + ny*dy.at<float>(j, i)) / (nx*nx*dxx.at<float>(j, i) + 2 * nx*ny*dxy.at<float>(j, i) + ny*ny*dyy.at<float>(j, i));

				if (fabs(t*nx) <= 0.5 && fabs(t*ny) <= 0.5)
				{
					Pt.push_back(i);
					Pt.push_back(j);
					//test:��ͼ������ʾ����
					cv::Point endpt = Point(i,j) + Point(nx * 10, ny * 10);
					//show.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 255, 0);
					cv::line(show, Point(i, j), Point(endpt.x, endpt.y), cv::Scalar(0, 255, 0));
				}
			}
		}
	}
}

/**
* @brief ������ͼ�����ϸ��,������
* @param srcΪ����ͼ��,��cvThreshold�����������8λ�Ҷ�ͼ���ʽ��Ԫ����ֻ��0��1,1������Ԫ�أ�0����Ϊ�հ�
* @param maxIterations���Ƶ���������������������ƣ�Ĭ��Ϊ-1���������Ƶ���������ֱ��������ս��
* @return Ϊ��srcϸ��������ͼ��,��ʽ��src��ʽ��ͬ��Ԫ����ֻ��0��1,1������Ԫ�أ�0����Ϊ�հ�
*/
cv::Mat thinImage(const cv::Mat & src, const int maxIterations)
{
	assert(src.type() == CV_8UC1);
	cv::Mat dst;
	int width = src.cols;
	int height = src.rows;
	src.copyTo(dst);
	int count = 0;  //��¼��������    
	while (true)
	{
		count++;
		if (maxIterations != -1 && count > maxIterations) //���ƴ������ҵ�����������    
			break;
		std::vector<uchar *> mFlag; //���ڱ����Ҫɾ���ĵ�    
		//�Ե���    
		for (int i = 0; i < height; ++i)
		{
			uchar * p = dst.ptr<uchar>(i);
			for (int j = 0; j < width; ++j)
			{
				//��������ĸ����������б��    
				//  p9 p2 p3    
				//  p8 p1 p4    
				//  p7 p6 p5    
				uchar p1 = p[j];
				if (p1 != 1) continue;
				uchar p4 = (j == width - 1) ? 0 : *(p + j + 1);
				uchar p8 = (j == 0) ? 0 : *(p + j - 1);
				uchar p2 = (i == 0) ? 0 : *(p - dst.step + j);
				uchar p3 = (i == 0 || j == width - 1) ? 0 : *(p - dst.step + j + 1);
				uchar p9 = (i == 0 || j == 0) ? 0 : *(p - dst.step + j - 1);
				uchar p6 = (i == height - 1) ? 0 : *(p + dst.step + j);
				uchar p5 = (i == height - 1 || j == width - 1) ? 0 : *(p + dst.step + j + 1);
				uchar p7 = (i == height - 1 || j == 0) ? 0 : *(p + dst.step + j - 1);
				if ((p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) >= 2 && (p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) <= 6)
				{
					int ap = 0;
					if (p2 == 0 && p3 == 1) ++ap;
					if (p3 == 0 && p4 == 1) ++ap;
					if (p4 == 0 && p5 == 1) ++ap;
					if (p5 == 0 && p6 == 1) ++ap;
					if (p6 == 0 && p7 == 1) ++ap;
					if (p7 == 0 && p8 == 1) ++ap;
					if (p8 == 0 && p9 == 1) ++ap;
					if (p9 == 0 && p2 == 1) ++ap;

					if (ap == 1 && p2 * p4 * p6 == 0 && p4 * p6 * p8 == 0)
					{
						//���    
						mFlag.push_back(p + j);
					}
				}
			}
		}

		//����ǵĵ�ɾ��    
		for (std::vector<uchar *>::iterator i = mFlag.begin(); i != mFlag.end(); ++i)
		{
			**i = 0;
		}

		//ֱ��û�е����㣬�㷨����    
		if (mFlag.empty())
		{
			break;
		}
		else
		{
			mFlag.clear();//��mFlag���    
		}

		//�Ե���    
		for (int i = 0; i < height; ++i)
		{
			uchar * p = dst.ptr<uchar>(i);
			for (int j = 0; j < width; ++j)
			{
				//��������ĸ����������б��    
				//  p9 p2 p3    
				//  p8 p1 p4    
				//  p7 p6 p5    
				uchar p1 = p[j];
				if (p1 != 1) continue;
				uchar p4 = (j == width - 1) ? 0 : *(p + j + 1);
				uchar p8 = (j == 0) ? 0 : *(p + j - 1);
				uchar p2 = (i == 0) ? 0 : *(p - dst.step + j);
				uchar p3 = (i == 0 || j == width - 1) ? 0 : *(p - dst.step + j + 1);
				uchar p9 = (i == 0 || j == 0) ? 0 : *(p - dst.step + j - 1);
				uchar p6 = (i == height - 1) ? 0 : *(p + dst.step + j);
				uchar p5 = (i == height - 1 || j == width - 1) ? 0 : *(p + dst.step + j + 1);
				uchar p7 = (i == height - 1 || j == 0) ? 0 : *(p + dst.step + j - 1);

				if ((p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) >= 2 && (p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9) <= 6)
				{
					int ap = 0;
					if (p2 == 0 && p3 == 1) ++ap;
					if (p3 == 0 && p4 == 1) ++ap;
					if (p4 == 0 && p5 == 1) ++ap;
					if (p5 == 0 && p6 == 1) ++ap;
					if (p6 == 0 && p7 == 1) ++ap;
					if (p7 == 0 && p8 == 1) ++ap;
					if (p8 == 0 && p9 == 1) ++ap;
					if (p9 == 0 && p2 == 1) ++ap;

					if (ap == 1 && p2 * p4 * p8 == 0 && p2 * p6 * p8 == 0)
					{
						//���    
						mFlag.push_back(p + j);
					}
				}
			}
		}

		//����ǵĵ�ɾ��    
		for (std::vector<uchar *>::iterator i = mFlag.begin(); i != mFlag.end(); ++i)
		{
			**i = 0;
		}

		//ֱ��û�е����㣬�㷨����    
		if (mFlag.empty())
		{
			break;
		}
		else
		{
			mFlag.clear();//��mFlag���    
		}
	}
	return dst;
}



/** \brief ��������ͼ��
  * \param[in] 
  * \param[out] 
  * \note  
  */
Mat getLaserImg(const cv::Size &imgsize, const int laserWidth, vector<int> &center_col, const int stripeType){
	Mat img = Mat::zeros(imgsize.height, imgsize.width, CV_8UC1);
	int _size = center_col.size();
	if (!_size) return img;
	if (!laserWidth % 2) return img;
	//todo::�������λ���Ƿ�Խ��
	if (stripeType == 1){
		if (_size < imgsize.height){
			for (int i = _size; i < imgsize.height; i++){
				center_col.push_back(center_col[_size - 1]);
			}
		}

		int hw = (laserWidth-1) / 2;
		for (int i = 0; i < center_col.size(); i++){
			for (int j = center_col[i] - hw; j <= center_col[i] + hw; j++){
				img.at<uchar>(i, j) = 255;
			}
		}
	}
	return img;
}


void on_mouse(int event, int x, int y, int flags, void *ustc)
{
	static Point pre_pt = (-1, -1);
	static Point cur_pt = (-1, -1);
	double imshowScaler = *(double*)ustc;
	if (event == CV_EVENT_LBUTTONDOWN)
	{
		org.copyTo(img);
		pre_pt = Point(x, y);
	}
	else if (event == CV_EVENT_MOUSEMOVE && (flags & CV_EVENT_FLAG_LBUTTON))//���������flagsΪ1 
	{
		img.copyTo(tmp);
		cur_pt = Point(x, y);
		rectangle(tmp, pre_pt, cur_pt, Scalar(255, 0, 0, 0), 5, 8, 0);
		imshow("img", tmp);
	}
	else if (event == CV_EVENT_LBUTTONUP)
	{
		org.copyTo(img);
		cur_pt = Point(x, y);
		rectangle(img, pre_pt, cur_pt, Scalar(0, 255, 0, 0), 5, 8, 0);
		imshow("img", img);
		img.copyTo(tmp);
		int width = abs(pre_pt.x - cur_pt.x);
		int height = abs(pre_pt.y - cur_pt.y);
		if (width == 0 || height == 0)
		{
			return;
		}
		roi = Rect(min(cur_pt.x, pre_pt.x), min(cur_pt.y, pre_pt.y), width, height);
		dst = org(roi);
		namedWindow("dst");
		imshow("dst", dst);
	}
}

Mat getHmat(string imagePath, const double cornerdis, const int cornerType, const Point2i cornerCount){

	double imshowScaler = 0.5;
	cv::namedWindow("img", WINDOW_AUTOSIZE & WINDOW_KEEPRATIO);
	setMouseCallback("img", on_mouse, &imshowScaler);
	org = imread(imagePath);
	cv::imshow("img", org);
	cv::resizeWindow("img", org.cols*imshowScaler, org.rows *imshowScaler);
	waitKey(0);

	//��ȡroi����Ľǵ�
	Size patternsize(cornerCount.x, cornerCount.y);
	vector<Point2f> corners;
	cvtColor(dst, dst, CV_RGB2GRAY);
	cv::GaussianBlur(dst, dst, Size(5, 5), 0, 0);


	bool patternfound = false;
	//Բ�ǵ���ȡ
	if (cornerType == 1){
		SimpleBlobDetector::Params params;
		params.maxArea = 10e5;
		params.minArea = 100;
		Ptr<FeatureDetector> blobDetector = SimpleBlobDetector::create(params);
		patternfound = findCirclesGrid(dst, patternsize, corners, CALIB_CB_SYMMETRIC_GRID, blobDetector);
	}
	if (cornerType == 2){
		patternfound = cv::findChessboardCorners(dst, patternsize, corners,
			CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE);
		if (patternfound){
			cornerSubPix(dst, corners, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 50, 0.1));
		} //���Ǳ���
	}
	//���̸�ǵ���ȡ

	for (int i = 0; i < corners.size(); i++){
		corners[i] += Point2f(roi.tl().x, roi.tl().y);
	}
	drawChessboardCorners(org, patternsize, Mat(corners), patternfound);

	//��ʾ��ȡ���
	cv::imshow("img", org);
	cv::resizeWindow("img", org.cols * imshowScaler, org.rows * imshowScaler);
	cv::waitKey(0);

	//��������ϵ�µ�����ֵ
	float step = cornerdis;
	vector<Point2f> corInWord;
	int _num = patternsize.width * patternsize.height;
	for (int i = 0; i < _num; i++){
		corInWord.push_back(Point2f(step*(i % patternsize.width), step*(i / patternsize.width)));
	}

	//���㵥Ӧ����(��src��dst�µı任����)
	return findHomography(corners, corInWord);
}