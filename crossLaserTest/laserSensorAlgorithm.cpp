#include "laserSensorAlgorithm.h"
#include "Algorithm.h"
#include "cvtool.h"
#include <fstream>
#include <list>


laserSensorAlgorithm::LaserExtractPara::LaserExtractPara()
{
	threasold = 60;
	windowWidth = 60;
	CAlgorithm::GaussKNew(guassK,5, 5, 1);

}

laserSensorAlgorithm::SensorPara::SensorPara()
{
	ratio = 1; 
	refLaserLine[0] = 0;
	refLaserLine[1] = 0;
	refLaserLine[2] = 0;
	hpara[0] = 0;
	hpara[1] = 0;
	hpara[2] = 0;
	refHmat = Mat::eye(3, 3, CV_64FC1);
	worldInHand = Mat::eye(3, 3, CV_64FC1);
}

laserSensorAlgorithm::laserSensorAlgorithm():
m_scanDir(ScanDir::ROW)
{
	m_para.imgHeight = 1200;
	m_para.imgWidth = 1600;
}

laserSensorAlgorithm::~laserSensorAlgorithm()
{
}

double laserSensorAlgorithm::getZ(Mat &src, const Rect &ROI)
{
	//***提取光条中心
	//if (!m_isCalibrated){
	//	return 0.0;
	//}
    LaserExtractPara _p = m_para.laserExtractPara;
	SensorPara _s = m_para;
	vector<Point2f> laserCenter;
	double _mean, _std;
	if (m_scanDir == ScanDir::ROW)
		CAlgorithm::LaserCenterDetect_row(src, _p.threasold, _p.windowWidth, 500, laserCenter, _mean, _std, 5, _p.guassK, 1);
	else if (m_scanDir == ScanDir::COL)
		CAlgorithm::LaserCenterDetect(src, _p.threasold, _p.windowWidth, 500, laserCenter, _mean, _std, 5, _p.guassK, 1);

	//test 
	//vector <Vec3f> tmpline;
	//fitLine_ransac(laserCenter, 0.1, 5, 0, tmpline);
	//cvtColor(src, src, CV_GRAY2RGB);
	//for (int i = 0; i < tmpline.size(); i++)
	//{
	//	Vec3f tarLine = tmpline[i];
	//	drawLine(src, tarLine);
	//}

	//***查找直线
	Mat lineImg = Mat::zeros(src.size(), CV_8UC1);
	Mat color_dst; 
	cvtColor(lineImg, color_dst, CV_GRAY2RGB);
	for (int i = 0; i < laserCenter.size(); i++){
		lineImg.at<uchar>(laserCenter[i].y, laserCenter[i].x) = 1;
	}
	vector<Vec4i> lines;
	HoughLinesP(lineImg, lines, 2, CV_PI / 90, 50, 15, 20);
	//for visual
	for (size_t i = 0; i < lines.size(); i++){
		line(color_dst, Point(lines[i][0], lines[i][1]),
			Point(lines[i][2], lines[i][3]), Scalar(0, 0, 255), 3, 8);
	}

	//***选择最远且和参考直线平行的一根直线作为目标直线
	Vec4i tarLine;
	vector <Vec4i> _lines;
	bool _f = false;
	float ang_threasold = 0.1;
	for (int i = 0; i < lines.size(); i++){
		double x = lines[i][2] - lines[i][0];
		double y = lines[i][3] - lines[i][1];
		double dfg = sqrt(x*x + y*y);
		if ((x*_s.refLaserLine[0] + y*_s.refLaserLine[1])/dfg < ang_threasold){
			_lines.push_back(lines[i]);
			_f = true;
		}
	}
	if (!_f) {
		cout << "no fit laser line!" << endl;
        return 0.0;
	}

	double  _max = -3000;
	for (int i = 0; i < _lines.size(); i++){
		Vec4i tarLine = _lines[i];
		double stripe_dis = tarLine[2] * _s.refLaserLine[0] + tarLine[3] * _s.refLaserLine[1] + _s.refLaserLine[2];
		_max = stripe_dis > _max ? stripe_dis : _max;
	}

	cout << "tarline dis to refline = "<<_max << endl;


	//todo::将拟合方法改为ranas的直线拟合方法
	//vector<double> tarLine;
	//cv::fitLine(laserCenter, tarLine, cv::DistanceTypes::DIST_L2, 0, 0.01, 0.01);
	////for visual 
	//Point2f star(tarLine[0] / tarLine[1] * (-tarLine[3]) + tarLine[2], 0);
	//Point2f end(tarLine[0] / tarLine[1] * (1000-tarLine[3]) + tarLine[2], 1000);
	//cv::line(src, star, end, Scalar(255, 0, 0), 2);

	//double stripe_dis = tarLine[2] * _s.refLaserLine[0] + tarLine[3] * _s.refLaserLine[1] + _s.refLaserLine[2];
	//cout << "testdist = " << stripe_dis << endl;
	return _s.polypara[0] * _max*_max + _s.polypara[1] * _max + _s.polypara[2];
	//return stripe_dis / _s.ratio;
}

Point2f laserSensorAlgorithm::getXY(const Point2f pointInImg, const Point2f tcp, const double z)
{
	if (!m_isCalibrated){
		return Point2f(0, 0);
	}
	//获取图像点在世界坐标系下的位置
	Mat hmat = getHmat(z);
	Mat imgp = (cv::Mat_<double>(3, 1) << pointInImg.x, pointInImg.y, 1);
	Mat_<double> _w = hmat * imgp;

	Mat wodp = (cv::Mat_<double>(4, 1) << _w(0, 0) / _w(2,0), _w(1, 0) / _w(2,0), 0, 1);
	Mat_<double> inTCP = m_para.worldInHand*wodp;
	return Point2f(inTCP(0, 0) + tcp.x, inTCP(1, 0) + tcp.y);
}

Point2f laserSensorAlgorithm::getXY(const Point2f pointInImg, const Point2f tcp)
{
	if (!m_isCalibrated){
		return Point2f(0, 0);
	}
	//获取图像点在世界坐标系下的位置
	Mat hmat = m_para.refHmat;
	Mat imgp = (cv::Mat_<double>(3, 1) << pointInImg.x, pointInImg.y, 1);
	Mat_<double> _w = hmat * imgp;

	Mat wodp = (cv::Mat_<double>(4, 1) << _w(0, 0) / _w(2, 0), _w(1, 0) / _w(2, 0), 0, 1);
	Mat_<double> inTCP = m_para.worldInHand*wodp;
	return Point2f(inTCP(0, 0) + tcp.x, inTCP(1, 0) + tcp.y);
}

cv::Point2f laserSensorAlgorithm::getOffsetToCenter(const cv::Point2f pointInImg)
{
	return getXY(pointInImg, Point2f(0, 0)) - getXY(Point2f(m_para.imgWidth / 2.0, m_para.imgHeight / 2.0), Point2f(0, 0));
}

bool laserSensorAlgorithm::readParameter(string filePath)
{
	ifstream ism(filePath, ios::in);
	//ifstream ism("E:/Projects/git/PCBAoi/LK-LSB-A_90/so9.mod", ios::in | ios::binary);

	boost::archive::text_iarchive ir(ism);
	m_para.serializeA(ir, 1);
	ism.close();
	m_isCalibrated = true;
	return true;
}

bool laserSensorAlgorithm::writeParameter(const string filePath)
{
	ofstream osm(filePath, ios::out);
	boost::archive::text_oarchive ar(osm);
	m_para.serializeA(ar, 1);
	osm.close();
	return true;
}

//bak: 采用拟合直线的方法
#if 0
bool laserSensorAlgorithm::calibrateZtoLaser(vector <cv::Mat> &laserImg, const vector<double> zdis)
{
    if (laserImg.size()!=4 || laserImg.size() != zdis.size())
	{
        return false;
	}
	double _mean, _std;
    LaserExtractPara _p = m_para.laserExtractPara;
	vector <vector <float>> lines;
	for (int i = 0; i < laserImg.size(); i++){
		vector<Point2f> stripes;
		vector<float> line;
		CAlgorithm::LaserCenterDetect_row(laserImg[i], _p.threasold, _p.windowWidth, 500, stripes, _mean, _std, 5, _p.guassK, 1);
		cv::fitLine(stripes, line, cv::DistanceTypes::DIST_L2, 0, 0.01, 0.01);
		lines.push_back(line);
	}

	//计算其他直线到第一条直线的距离：
	m_para.refLaserLine[0] = 1/lines[0][0];
	m_para.refLaserLine[1] = -1 / lines[0][1];
	m_para.refLaserLine[2] = lines[0][3] / lines[0][1] - lines[0][2] / lines[0][0];
	double  dis = sqrt(m_para.refLaserLine[0] * m_para.refLaserLine[0] + m_para.refLaserLine[1] * m_para.refLaserLine[1]);
	m_para.refLaserLine[0] /= dis;
	m_para.refLaserLine[1] /= dis;
	m_para.refLaserLine[2] /= dis;
	vector <float> diss, _zdis;
	for (int i = 1; i < lines.size(); i++)
	{
		float stripe_dis = m_para.refLaserLine[0] * lines[i][2] + m_para.refLaserLine[1] * lines[i][3] + m_para.refLaserLine[2];
		diss.push_back(stripe_dis);
		_zdis.push_back(zdis[i]);
		cout << "diss = " << stripe_dis << endl;
	}

	Mat vmat = (Mat_<double>(3, 3) << diss[0] * diss[0], diss[0], 1, diss[1] * diss[1], diss[1], 1, diss[2] * diss[2], diss[2], 1);
	Mat ymat = (Mat_<double>(3, 1) << _zdis[0], _zdis[1], _zdis[2]);

	invert(vmat, vmat);
	Mat para = vmat * ymat;
	m_para.polypara[0] = para.at<double>(0, 0);
	m_para.polypara[1] = para.at<double>(1, 0);
	m_para.polypara[2] = para.at<double>(2, 0);
    return true;
}
#endif


bool laserSensorAlgorithm::calibrateZtoLaser(vector <cv::Mat> &laserImg, const vector<double> zdis)
{
	if (laserImg.size() != 4 || laserImg.size() != zdis.size())
	{
		return false;
	}
	double _mean, _std;
	LaserExtractPara _p = m_para.laserExtractPara;
	vector <float> line;
	vector <vector <Point2f>> stripes;
	for (int i = 0; i < laserImg.size(); i++){
		vector<Point2f> stripe;
		if (m_scanDir == ScanDir::ROW)
			CAlgorithm::LaserCenterDetect_row(laserImg[i], _p.threasold, _p.windowWidth, 500, stripe, _mean, _std, 5, _p.guassK, 1);
		else if (m_scanDir == ScanDir::COL)
			CAlgorithm::LaserCenterDetect(laserImg[i], _p.threasold, _p.windowWidth, 500, stripe, _mean, _std, 5, _p.guassK, 1);
		//第一张图片所提取的条纹直线作为参考直线
		if (i == 0){
			cv::fitLine(stripe, line, cv::DistanceTypes::DIST_L2, 0, 0.01, 0.01);
		}
		else
			stripes.push_back(stripe);
	}

	//计算其他直线到第一条直线的距离：
	m_para.refLaserLine[0] = 1 / line[0];
	m_para.refLaserLine[1] = -1 / line[1];
	m_para.refLaserLine[2] = line[3] / line[1] - line[2] / line[0];
	double  dis = sqrt(m_para.refLaserLine[0] * m_para.refLaserLine[0] + m_para.refLaserLine[1] * m_para.refLaserLine[1]);
	m_para.refLaserLine[0] /= dis;
	m_para.refLaserLine[1] /= dis;
	m_para.refLaserLine[2] /= dis;
	vector <float> diss, _zdis;
	for (int i = 0; i < stripes.size(); i++)
	{
		double _sum = 0;
		for (int j = 0; j < stripes[i].size(); j++){
			_sum += m_para.refLaserLine[0] * stripes[i][j].x + m_para.refLaserLine[1] * stripes[i][j].y + m_para.refLaserLine[2];
		}
		_sum /= stripes[i].size();
		diss.push_back(_sum);
		_zdis.push_back(zdis[i + 1]);
		cout << "diss = " << _sum << endl;
	}

	Mat vmat = (Mat_<double>(3, 3) << diss[0] * diss[0], diss[0], 1, diss[1] * diss[1], diss[1], 1, diss[2] * diss[2], diss[2], 1);
	Mat ymat = (Mat_<double>(3, 1) << _zdis[0], _zdis[1], _zdis[2]);

	invert(vmat, vmat);
	Mat para = vmat * ymat;
	m_para.polypara[0] = para.at<double>(0, 0);
	m_para.polypara[1] = para.at<double>(1, 0);
	m_para.polypara[2] = para.at<double>(2, 0);
	return true;
}

void laserSensorAlgorithm::setLaserRefPlane(Mat refLaserImg)
{
	vector<Point2f> stripes;
	vector<float> line;
	LaserExtractPara _p = m_para.laserExtractPara;
	double _mean, _std;
	CAlgorithm::LaserCenterDetect(refLaserImg, _p.threasold, _p.windowWidth, 500, stripes, _mean, _std, 5, _p.guassK, 1);
	cv::fitLine(stripes, line, cv::DistanceTypes::DIST_L2, 0, 0.01, 0.01);

	m_para.refLaserLine[0] = 1 / line[0];
	m_para.refLaserLine[1] = -1 / line[1];
	m_para.refLaserLine[2] = line[3] / line[1] - line[2] / line[0];
	double  dis = sqrt(m_para.refLaserLine[0] * m_para.refLaserLine[0] + m_para.refLaserLine[1] * m_para.refLaserLine[1]);
	m_para.refLaserLine[0] /= dis;
	m_para.refLaserLine[1] /= dis;
	m_para.refLaserLine[2] /= dis;
	return;
}

void laserSensorAlgorithm::ratioxyFromZ(const vector<Mat> hmats, const vector<double> z, double &ratio, Point2d &dir)
{
	if (hmats.size() != z.size() || z.size() < 2) return;
	int n = z.size();
	Mat hmat0_inv;
	invert(hmats[0], hmat0_inv);
	vector<Point2d> pts;
	vector <double> _ratio;
	for (int i = 1; i < n; i++){
		Mat_<double> tr = hmat0_inv*hmats[i];
#ifdef _DEBUG
		cout << tr << endl;
#endif // _DEBUG
		double dis = sqrt(pow(tr(0, 2), 2) + pow(tr(1, 2), 2));
		_ratio.push_back(dis / (z[i] - z[0]));
		Point2d pt(tr(0, 2) / dis, tr(1, 2) / dis);
		pts.push_back(pt);
	}

#if _DEBUG
	cout << "all the ratio: ";
	for (int i = 0; i < _ratio.size(); i++){
		cout << _ratio[i] << " ";
	}
	cout << endl;

	cout << "all the point: " << endl;
	for (int i = 0; i < pts.size(); i++){
		cout << pts[i] << endl;
	}
#endif
	ratio = cv::sum(_ratio)[0] / _ratio.size();
	Scalar sr = cv::sum(pts);
	dir = Point2d(sr[0] / pts.size(), sr[1] / pts.size());
	m_para.hpara[0] = ratio;
	m_para.hpara[1] = dir.x;
	m_para.hpara[2] = dir.y;
	m_para.refHmat = hmats[0];
	return;
}

Mat laserSensorAlgorithm::getHmat(double Z)
{
	SensorPara _p = m_para;
	double dis = Z*_p.hpara[0];
	Mat tran = Mat::eye(3, 3, CV_64FC1);
	tran.at <double>(0, 2) = dis * _p.hpara[1];
	tran.at <double>(1, 2) = dis * _p.hpara[2];
	return _p.refHmat * tran;
}

//从0....n-1中随机等概率的输出m个不重复的数
vector<int> knuth(int n, int m)
{
	vector<int> outn;
	srand((unsigned int)time(0));
	for (int i = 0; i < n; i++) {
		if (rand() % (n - i) < m) {
			outn.push_back(i);
			m--;
		}
	}
	return outn;
}


Vec3f getline_2p(const Point2f &p1, const Point2f &p2)
{
	Vec3f line;
	float A = -(p2.y - p1.y) / (p2.x - p1.x);
	float nor = sqrt(A*A + 1);
	line[0]=(A / nor);
	line[1]=(1 / nor);
	line[2]=(-(p1.y + A*p1.x) / nor);
	return line;
}

void drawLine(Mat &src, const Vec3f &line)
{
	Vec3f tarLine = line;
	Point2f p1(0, -tarLine[2] / tarLine[1]);
	Point2f p2(-tarLine[2] / tarLine[0], 0);
	Point2f star;
	if (p1.y > 0 && p1.y < src.cols) star = p1;
	if (p2.x > 0 && p2.x < src.rows) star = p2;
	int row = src.rows - 1;
	int col = src.cols - 1;
	Point2f p3(col, -(tarLine[0] * col + tarLine[2]) / tarLine[1]);
	Point2f p4(-(tarLine[1] * row + tarLine[2]) / tarLine[0], row);
	Point2f end;
	if ( p3.y > 0 &&  p3.y < src.cols) end = p3;
	if ( p4.x > 0 &&  p4.x < src.rows) end = p4;
	cv::line(src, star, end, Scalar(255, 0, 0), 2);
}

Vec3f getline_pd(const Point2f &pt, const Point2f &dir)
{
	Vec3f para;
	para[0] = 1 / dir.x;
	para[1] = -1 / dir.y;
	para[2] = pt.y / dir.y - pt.x / dir.x;
	double  dis = sqrt(para[0] * para[0] + para[1] * para[1]);
	para[0] /= dis;
	para[1] /= dis;
	para[2] /= dis;
	return para;
}

//srcPnts 输入点；minIneRatio 最小内点比例；disThreasold 视为内点的距离阈值；minDis ransac的最小采样距离
bool fitLine_ransac(const vector<Point2f> &srcPnts, double minIneRatio, double disThreasold, double minDis, vector <Vec3f> &lines){
	clock_t t1 = clock();
	if (srcPnts.size() < 2)
		return false;
	float inerDisThreasold = disThreasold;
	int ors = srcPnts.size();
	list <Point2f> listPnts;
	std::copy(srcPnts.begin(), srcPnts.end(), back_inserter(listPnts));
	int failedTimes = 0;
	while (true)
	{
		int lis = listPnts.size();
		//退出循环条件：1）list为空；2）剩下的点少于原来点数的百分之5；3）拟合失败次数n > 100
		if (lis < 2 || lis / (float)ors < minIneRatio || failedTimes > 50)
			break;
		//从list中随机选出两个点计算直线方程
		vector <int> _id = knuth(lis, 2);
		cout << _id[0] << " " << _id[1] << endl;
		Point2f p1, p2;
		list <Point2f>::iterator itr = listPnts.begin();
		for (int i = 0; itr != listPnts.end(); itr++, i++)
		{
			if (i == _id[0]) p1 = *itr;
			if (i == _id[1]) p2 = *itr;
			if (i > max(_id[0],_id[1])) break;
		}
		Vec3f line = getline_2p(p1, p2);

		//遍历所有点，获取内点
		itr = listPnts.begin();
		vector <Point2f> inlinePnt;
		while (itr != listPnts.end())
		{
			float dis = itr->x*line[0] + itr->y*line[1] + line[2];
			if (fabs(dis) < inerDisThreasold)
			{
				inlinePnt.push_back(*itr);
				itr = listPnts.erase(itr);
				//todo::加入内点，重新再拟合
				vector<float> _line;
				cv::fitLine(inlinePnt, _line, cv::DistanceTypes::DIST_L2, 0, 0.01, 0.01);
				line = getline_pd(Point2f(_line[2], _line[3]), Point2f(_line[0], _line[1]));
			}
			else
				itr++;
		}
		if (inlinePnt.size() / (float)ors < minIneRatio){
			failedTimes++;
		}
		else{
			lines.push_back(line);
			failedTimes = 0;
		}
	}
	cout << "line ransac fit time: "<<clock() - t1 << endl;
	return true;
}

int getZInDevice(double Z)
{
	int pulse = Z / pulseDis;
	return deviceZatRef + pulse;
}

double DistPtToLine(cv::Point2f& PtTest, cv::Point2f& p1, cv::Point2f p2)
{
	//计算ABC

	Vec3f line;
	float A = -(p2.y - p1.y) / (p2.x - p1.x);
	float nor = sqrt(A*A + 1);
	line[0] = (A / nor);
	line[1] = (1 / nor);
	line[2] = (-(p1.y + A*p1.x) / nor);

	float _A = line[0];
	float _B = line[1];
	float _C = line[2];

	if ((p1.x - p2.x) != 0)
	{
		return (abs(_A*PtTest.x + _B*PtTest.y + _C)) / (sqrt(_A*_A + _B*_B));
	}
	else
	{
		return abs(PtTest.x - p1.x);
	}

}


bool PtContain(cv::Point2f& Pt, cv::Point2f& LUp, cv::Point2f RUp, cv::Point2f& RDown, cv::Point2f& LDown)
{
	double toL = DistPtToLine(Pt, LUp, LDown);
	double toR = DistPtToLine(Pt, RUp, RDown);
	double toU = DistPtToLine(Pt, LUp, RUp);
	double toB = DistPtToLine(Pt, LDown, RDown);

	double W = sqrt((LUp.x - RUp.x)*(LUp.x - RUp.x) + (LUp.y - RUp.y)*(LUp.y - RUp.y));
	double H = sqrt((LUp.x - LDown.x)*(LUp.x - LDown.x) + (LUp.y - LDown.y)*(LUp.y - LDown.y));

	if ((abs(toL - toR) < W) && (abs(toU - toB) < H))
	{
		return true;
	}
	else
	{
		return false;
	}
}