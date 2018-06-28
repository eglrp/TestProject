
#pragma once
//***用于检测内存泄漏函数头
//#ifdef _DEBUG  
//#define DEBUG_CLIENTBLOCK new( _CLIENT_BLOCK, __FILE__, __LINE__)  
//#else  
//#define DEBUG_CLIENTBLOCK  
//#endif  // _DEBUG  
//#define _CRTDBG_MAP_ALLOC  
//#include <stdlib.h>  
//#include <crtdbg.h>  
//#ifdef _DEBUG  
//#define new DEBUG_CLIENTBLOCK  
//#endif  // _DEBUG 

//***c++header
#include <iostream>
#include <string>
#include <stdio.h>
#include <Winsock2.h>
#include <socketapi.h>
#include <fstream>
#include <time.h>
#include <random>
#include <math.h>
#include "XMLReader.h"
#include "CoreAlgorithm.h"
#include<vector>
#include<iterator>
#include "SerialPort.h"
#include <queue>
#include <thread>
#include <windows.h>
#include <xtree>

#include <omp.h>
#include <intrin.h>
#include "person.h"

//***opencv header
#include "cv.h"
#include "highgui.h"
#include "cxcore.h"
//#include "feature2d.h"
#include "opencv.hpp"//hpp file equal the .h + .cpp, the fun decalare and implement in the same file.

//***qt header
#include <QStringList>
#include <QImageReader>
#include <QXmlStreamWriter>
#include <QXmlStreamReader>
#include <QDomDocument>
#include <QFile>
#include <QtCore/QCoreApplication>
#include <QFileSystemWatcher>
#include <QDir>

//other header
//#include <Eigen/dense>
//#include "EllipseFitting.h"
//using namespace Eigen;

//#define HALCON
//#define QTTEST
#define CTEST
//#define OPENCVTEST

using namespace cv;
using namespace std;
//using namespace EllipseFitting;

#pragma region 函数声明
void testfun2(const vector<float>& input_1, float input_2, vector<float>& out);
void testfun1(vector<float>&input_1, float input_2);
bool ReadASCII(const char *cfilename, vector<Point3f> &points);
bool ReadSTLFile(const char *cfilename, vector<Point3f> &points);
bool ReadBinary(const char *cfilename, vector<Point3f> &points);
void on_trackbar(int,void*);
void ProduceRandNum(int Min, int Max, int num, vector <int> &outNum);
vector<int> getRandom(int Min, int Max, int total);
bool onEllipse(const RotatedRect rotRect, const Point pnt);
void testfun(string * &);
//template <typename T>
//T SumN(const T &a, const T &b, T &sumValue);
//template<class T>
//int find(vector<T> &vec, T data);
#pragma endregion 函数声明
//全局变量定义
Mat img, outimg;
int word = 0;
HANDLE mute;
DWORD WINAPI ACC(LPVOID pnt);
DWORD WINAPI muliThreadTest(LPVOID pnt);
void outtime(int outTime);

string stringGloble = "im global";
#pragma region 类定义

struct teststru
{
	int a = 0;
	int b = 1;
};

class testca
{
public:
	testca(){ a = 3; }
	~testca(){}
public:
	int a;
	int b;
private:

};
//基类
class baseClass
{
public:
	baseClass()
	{
		cout << "调用了基类的构造函数！" << endl;
	}
	~baseClass()
	{	}
public:
	int memberA;
public:
	virtual int caculate(int num)
	{
		memberA = 0;
		return 0;
	}
	//virtual int caculate(int  num) = 0;
};
class baseClassb
{
	baseClassb(){ m_b = 1; }
	~baseClassb(){}
public:
	int m_b;
};

class childA :baseClass
{
public:
	childA()
	{
		m_ca = 2;
		cout << "调用了子类childA的构造函数！" << endl;
	}
	int caculate(int num)
	{
		num = 0;
		memberA = 1;
		return num + 1;
	}
	int m_ca;
};
class childB :baseClass
{
public:
	const int staticNum;
public:
	childB() :staticNum(0)
	{
		cout << "调用了子类ChildB的构造函数！" << endl;
		memberA = 5;
	}
	int caculate(int num)
	{
		num = 0;
		memberA = 5;
		return num + 5;
	}

};


  



#pragma endregion 类定义


//自定义map的比较函数
struct cmp_key
{
	bool operator()(const Vec4i &k1, const Vec4i &k2)const
	{
		if (k1[0] != k2[0])
		{
			return k1[0] < k2[0];
		}
		if (k1[1] != k2[1])
		{
			return k1[1] < k2[1];
		}
		if (k1[2] != k2[2])
		{
			return k1[2] < k2[2];
		}
		if (k1[3] != k2[3])
		{
			return k1[3] < k2[3];
		}
		return false;
	}
};

int main(int argc, char *argv[])
{
#pragma region QT test
#ifdef QTTEST
	QCoreApplication a(argc, argv);
	//qstringlist test
#if 0
	QStringList strlist;
	strlist.push_back("NIKE");
	strlist.push_back("ANTA");
	strlist.push_back("ADIDAS");
	strlist.push_back("MEIKE");
	for (int i = 0; i < strlist.size();i++)
	{
		cout << strlist[i].toStdString() << endl;
	}
#endif
#if 0 //qt文件监视器
	QFileSystemWatcher fwatch;
	fwatch.addPath("")
#endif
	//xml file read and write
#if 0
	QFile file("D:\vs12program\MFC_OneEyeMeasurement_6\datafile\JointPluse.xml");
	if (!file.open(QFile::ReadOnly))
	{
		cout << "file read error" << endl;
	}
	QXmlStreamReader *xmlreader = new QXmlStreamReader();
	QXmlStreamWriter *xmlwriter = new QXmlStreamWriter();
	xmlreader->setDevice(&file);
	//while (!xmlreader->atEnd())
	//{
	//	xmlreader->readNext();

	//}
#endif
#if 0 //删除文件夹和添加文件夹
	QDir file;
	if (!file.mkpath("../datafile")) //常见路径，如果路径存在，返回true
	{
		std::cout << "failed!" << endl;
	}
	if (!file.mkdir("../datafile/new")) //创建文件夹，如果已经存在，返回false
	{
		std::cout << "failed!" << endl;
	}
	if (!file.rmdir("../datafile/new")) //删除文件夹，文件夹必须为非空
	{
		std::cout << "failed!" << endl;
	}
#endif
	return a.exec();
#endif // QTTEST
#pragma endregion QT test

#pragma region C++test
#ifdef CTEST
	//dll test
#if 1
	person ps("hello zzl");
	cout << ps.name << endl;
	cout << ps.getnum() << endl;
#endif

	//lib test

#if 1


#endif

	//
#if 0
	char buf[1000];
	GetCurrentDirectoryA(1000, buf); //得到当前工作路径 
	cout << buf << endl;
	char strModule[256];
	GetModuleFileNameA(NULL, strModule, 256); //得到当前模块路径 
	cout << strModule << endl;
	Sleep(10000);
#endif
	//测试vector和数组操作的时间对比
#if 0
	clock_t t1 = clock();
	double pnts[1280];
	for (int i = 0; i < 2000; i++)
	{
		for (int j = 0; j < 1280; j++)
		{
			if (j>=0)
				pnts[j] = (20.30*2/4+pnts[0])/2;
		}
	}
	clock_t t2 = clock();
	vector<double> pntv;
	//pntv.reserve(1300); 
	for (int i = 0; i < 2000; i++)
	{
		pntv.clear();
		for (int j = 0; j < 1280; j++)
		{
			//pntv[j] = 20.30;
			pntv.push_back(20.30);
		}
	}
	clock_t t3 = clock();
	cout << t2 - t1 << " vecotr" << t3 - t2 << endl;
	/*两者速度差了十倍,改变数组中的计算复杂度，时间并没有因此差很多*/
#endif
	//openmp使用测试
#if 0
	//vector<int>  dfsd;
	//cout << dfsd.capacity() << endl;
	//dfsd.reserve(1000);
	//cout << dfsd.capacity() << endl;
	//cout << dfsd.size() << endl;
	int a = 0;
#pragma omp parallel if (false) num_threads(6) //这里if表示的意思，是如果条件成立，进行并行，如果不成立则正常执行
	{
		std::cout << omp_get_thread_num();
	}
	float in_2 = 9;
	vector<float> in_1(1000000,0), out;
	clock_t t1 = clock();
	//for (int i = 0; i < 1000; i++)
	//{
	//	testfun1(in_1, in_2); 
	//}	
	out.reserve(1000000);
	for (int i = 0; i < 1000; i++)
	{
		out.clear();
		testfun2(in_1, in_2, out); //换成testfun2，时间变成>100s
	}
//#pragma omp parallel for // openmp + sse
//		for (int i = 0; i < 1000; i++)
//		{
//			testfun1(in_1, in_2);
//		}
	clock_t t2 = clock();
	cout << t2 - t1 <<endl; //sse 7.8s, sse(无内存拷贝)：1.9s, openmp 1.8s(//换成testfun2，时间变成>100s), 无加速 7s, sse(无内存拷贝)+openmp:0.5s
	int dfd = 0;
#endif
	//cpu加速测试
#if 0
	float in_2 = 9;
	vector<float> in_1(1000000,0), out;
	clock_t t1 = clock();
	//for (int i = 0; i < 1000; i++)
	//{
	//	testfun1(in_1, in_2);
	//}
	// openmp + sse
//#pragma omp parallel for
	for (int i = 0; i < 1000; i++)
	{
		testfun1(in_1, in_2);
	}
	clock_t t2 = clock();
	cout << t2 - t1 <<endl; //sse 7.8s, sse(无内存拷贝)：1.9s, openmp 1.8s, 无加速 7s, sse(无内存拷贝)+openmp:0.5s
#endif

#if 0 //类指针指向问题
	baseClass ea; 
	int *inta = (int *)&ea;
	cout << "---------------------" << endl;
	cout << inta[0] << endl;
	cout << inta[3] << endl;

	int a[2] = {1,2};
	int *ap = a;
	cout << ap[0]<<endl;

	teststru st;
	int *intb = (int *)&st;
	cout << intb[0] << endl;
	/*
	总结：当类中存在虚函数时，第一个指针指向类的虚函数表（vtable)，然后当类存在继承时指针首先指向父类中定义的成员；
	vs查看内存分布的方法：选择左侧的C/C++->命令行，然后在其他选项这里写上/d1 reportAllClassLayout，它可以看到所有相关类的内存布局，如果
	写上/d1 reportSingleClassLayoutXXX（XXX为类名），则只会打出指定类XXX的内存布局。近期的VS版本都支持这样配置。
	*/

#endif

#if 0 //stringstream测试
	stringstream str;
	str << 252;
	int nm;
	str >> nm;
	cout << nm<< endl;
#endif

#if 0
	int ee[] = { 0, 1 };
	vector<int> dd(ee, ee + 1);
	int i = 2;
	int j = i << 3;
	cout << i << " " << j << endl;
#endif

	cv::Mat rt = cv::Mat::eye(3, 3, CV_64FC1);
	cv::Mat pose = cv::Mat::zeros(6,1, CV_64FC1);
	cv::Rodrigues(rt,rt);
	rt.copyTo(pose.rowRange(0,3).col(0));
	cout << pose<<endl;
	Mat rt1 = (Mat_<double>(3,1)<<1,2,3);  
	cv::normalize(rt1,rt1);
	cout << norm(rt1) << endl;
	//指针参数
#if 0
	string *str = new string("im local");
	testfun(str);
	cout << *str << endl;
	*str = "im global changed";
	cout << stringGloble << endl;
#endif



	//实验map
#if 0
	/*map在进行插入的时候是不允许有重复的键值的，如果新插入的键值与原有的键值重复则插入无效，可以通过insert的返回值来判断是否成功插入。下面是insert的函数原型：
	  pair<iterator, bool> insert(const value_type& x);
	  可以通过返回的pair中第二个bool型变量来判断是否插入成功。*/
	std::map<string, int> maptest;
	for (int i = 0; i < 200;i++)
	{
		stringstream st;
		st << 0;
		maptest.insert(map<string, int>::value_type(st.str(), i)); //键值必须是唯一的标识，如果键值一样，map中会自动覆盖
	}
	//maptest.insert(map<string, int>::value_type("b", 2));
	//maptest.insert(map<string, int>::value_type("zzl", 1));
	//maptest.insert(map<string, int>::value_type("a", 2));
	//cout << maptest["b"] << endl;
	//cout << maptest["zzl"] <<endl;
	//
	std::map <cv::Vec4i, vector<Point2i>, cmp_key> mashtable;
	std::pair<map <cv::Vec4i, vector<Point2i>, cmp_key> ::iterator, bool > ret;
	for (int i = 0; i < 1000; i++)
	{
		cv::Vec4i t = { i * 1, i * 2, i * 3, i * 4 };
		//if (i == 4)
		//{
		//	t = Vec4i(3 * 1, 3 * 2, 3 * 3, 3 * 4);
		//}
		if (i == 4)
		{
			t = Vec4i(3 * 1, 3 * 2, 3 * 3, 4 * 4); //键值只要不完全相同就可成功插入键值
		}
		vector <Point2i> pairs;
		Point2i ppair1 = { i, i };
		pairs.push_back(ppair1);
		ret = mashtable.insert(map<cv::Vec4i, vector<Point2i>>::value_type(t, pairs));
		if (!ret.second)
		{
			mashtable[t].push_back(ppair1);
		}
	}
	cv::Vec4i t = { 3 * 1, 3 * 2, 3 * 3, 3 * 4 };
	cout << mashtable[t] << endl;

	cv::Vec4i t2 = { 1000 * 1, 1000 * 2, 1000 * 3, 1000 * 4 };
	cout << mashtable[t2] << endl;
#endif

	//进程操作
#if 0
	STARTUPINFO startupinfo = { 0 };
	PROCESS_INFORMATION processInformation = { 0 };
#endif

	//读取txt中的点
#if 0
	ifstream pnts;
	pnts.open("pnt.txt");
	if (!pnts.is_open())
	{
		cout << "读取文件错误！"<< endl;
		return 1;
	}
	while (!pnts.eof())
	{
		Point3f pnt3d;
		pnts >> pnt3d.x >> pnt3d.y >> pnt3d.z;
		cout << pnt3d << endl;
	}
#endif

	//测试消息队列
#if 0
		SendMessage
#endif

	//测试cpu的并行计算:开多个线程并没有提高CPU的利用率，当然也不用提高计算速度
#if 0
	for (int i = 0; i < 8; i++)
	{
		cout << i <<"线程开始计算" << endl;
		DWORD id;
		HANDLE da = CreateThread(NULL, 0, muliThreadTest, NULL, 0, &id);
		DWORD_PTR r = DWORD_PTR(i);
		SetThreadAffinityMask(da, DWORD_PTR(i));
		if (da == NULL)
		{
			printf("CreateThread error: %d\n", GetLastError());
		}
	}

	while (true)
	{
		int j = 0;
	}
#endif

	//测试二维数组的指针访问方式
#if 0
	double da2d[2][2] = { { 1, 2 }, { 3, 4 } }; //二维数组中 da2d[*]表示*行的首地址 
	double (*pn)[2] = da2d;
	cout << **da2d << endl;
	cout << *(da2d[0] + 2) << endl;

#endif
	//测试队列
#if 0
	queue <int> da;
	for (int i = 0; i < 10; i++)
	{
		da.push(i);
	}
	for (int i = 0; i < 10; i++)
	{
		cout <<da.front()<<endl;
		int *tem = &da.front();
		da.pop();
		cout <<"tem = "<< *tem << endl;
	}
#endif
	//测试带超时的函数
#if 0
	outtime(1000);
	cout << "超时！" << endl;
	outtime(5000);
	cout << "超时！" << endl;
#endif
#if 0
	int data[4][4] = { { 80, 75, 92,34 }, { 61, 65, 71,67 }, { 59, 63, 70,67 }, { 85, 87, 90,67 } };
	Mat matdat = Mat::eye(4, 4, CV_8UC1);
	uchar *prt = matdat.data;
	for (int i = 0; i < matdat.cols; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			cout << (int)*(prt + i*4+ j) << endl;
		}
	}

#endif
	//测试基类使用子类的方法的编程方式
#if 0
	baseClass *bc;
	childA *ca = new childA();
	childB *cb = new childB();
	//cout << bc->caculate(5) << endl;
	//cout << bc->memberA<< endl;
	bc = (baseClass *)ca;
	cout << bc->caculate(5) << endl;
	cout << bc->memberA << endl;
	bc = (baseClass *)cb;
	cout << bc->caculate(5) << endl;
	cout << bc->memberA << endl;
#endif
#if 0
	cout << 0 % 2 << endl;
	cout << 1 % 2 << endl;
	cout <<3%2<<endl;
	cout << 2%2<<endl;
#endif
#if 0 //仿真机器人力传感器
	CSerialPort com;
	if (!com.InitPort(2))
	{
		cout << "init port error!" << endl;
		return false;
	}
	char cr;
	while (true)
	{
		if (com.ReadChar(cr))
		{
			if (cr == 'F')
			{
				unsigned char forceValue;
				cout << "请输入力值大小：" << endl;
				cin >> forceValue;
				if (!com.WriteData(&forceValue, 1))
					cout << "写入数据失败！" << endl;
			}
		}
		else
		{
			cout << "读取错误" << endl;
		}
	}
#endif
#if 0//测试串口类
	CSerialPort com;
	if (!com.InitPort(2))
	{
		cout << "init port error!" << endl;
		return false;
	}
	cout << com.GetBytesInCOM() << endl;
	char cr[8];
	while (true)
	{
		com.ReadChar(cr[0]);
	}
	/*while (true)
	{
	int num = com.GetBytesInCOM();
	if (num != 0)
	{
	Sleep(10);
	int num = com.GetBytesInCOM();
	char rt[200];
	for (int i = 0; i < num; i++)
	{
	com.ReadChar(rt[i]);
	cout << rt[i];
	}
	cout <<endl;
	}
	}*/
#endif
#if 0 //线程锁测试
	mute = CreateMutex(NULL, false, NULL);
	if (mute == NULL)
	{
		printf("CreateMutex error: %d\n", GetLastError());
		return 1;
	}
	HANDLE Hthread[5];
	for (int i = 0; i < 5; i++)
	{
		DWORD thisID;
		Hthread[i] = CreateThread(NULL, 0, ACC, NULL, 0, &thisID);
		if (Hthread[i] == NULL)
		{
			printf("CreateThread error: %d\n", GetLastError());
			return 1;
		}
		CloseHandle(Hthread[i]); //线程为执行结束前关闭线程句柄不会影响线程的执行。
	}
	//等待所有线程执行结束
	//DWORD re = WaitForMultipleObjects(5, Hthread, TRUE, INFINITE); //同步函数，等线程结束后才会返回，这里有个重要的概念叫
	//// signaled state, 该函数就是根据这个信号状态判断线程是否结束运行。
	//if (re == WAIT_FAILED)
	//{
	//	printf("wait for multiply object failed!: %d\n", GetLastError());
	//}
	string str;
	std::cin >> str;
	CloseHandle(mute); //在所有线程都执行结束后才能关闭，否则不起作用
	system("pause");
#endif
	//测试vector =操作的拷贝形式
#if 0 
	//stringstream str;
	//str << "the smooth params is：\n" << "zheng zelong";
	//cout << str.str() << endl;
	string str;
	str.append("zhengzlong\n");
	str.append("dfsdfsdfss");
	cout << str << endl;
	vector <string> strs1;
	vector <string> strs2;
	for (int i = 0; i < 100000; i++)
	{
		strs1.push_back("hello word!");
	}
	strs2 = strs1; //等号操作是完全拷贝！
	strs1.clear();
	//for (int i = 0; i < 100; i++)
	//{
	//	cout<< strs2[i]<<endl;
	//}
	cin >> strs2[1];
#endif
	//内存泄漏检测测试
#if 0 
	class C
	{
	public:
		C(){ itr = 12; dr = 35.2; str = "sdfsdfsdf"; }
		~C() {}
	public:
		int itr;
		double dr;
		string str;
	};
	//_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF|_CRTDBG_LEAK_CHECK_DF); 
	//***设置内存泄漏提示
	int tmpDbgFlag;
	_CrtSetReportMode(_CRT_ERROR, _CRTDBG_MODE_FILE);
	_CrtSetReportFile(_CRT_ERROR, _CRTDBG_FILE_STDERR);
	/*
	* Set the debug-heap flag to keep freed blocks in the
	* heap's linked list - This will allow us to catch any
	* inadvertent use of freed memory
	*/
	tmpDbgFlag = _CrtSetDbgFlag(_CRTDBG_REPORT_FLAG);
	tmpDbgFlag |= _CRTDBG_DELAY_FREE_MEM_DF;
	tmpDbgFlag |= _CRTDBG_LEAK_CHECK_DF;
	_CrtSetDbgFlag(tmpDbgFlag);
	for (int i = 0; i < 100; i++)
	{
		C *cc = new C();
	}
	int i = 0;
	return 0;
#endif
	//测试通过管道传输地址，获取
#if 0 
	
#endif
	//测试用通过传递地址字符串实现变量的赋值
#if 0 
	struct S
	{
		int s1;
		double s2;
		string name;
	};
	S s;
	s.name = "zzl";
	s.s1 = 25;
	s.s2 = 623.2;
	S *sp;
	DWORD DW;
	DW = (DWORD)&s.s1;
	cout << DW << endl;
	cout << &s.s1 << endl;
	sp = (S *)&s.s1; //只要传递首地址便可传递整个结构体
	cout << sp->name << endl;
	cout << sp->s2 << endl;
#endif
	//模板函数测试
#if 0 
	int a = 9;
	int b = 10;
	int c;
	SumN(a, b, c);
	cout << "c = " << c << endl;
	float af = 9;
	float bf = 10;
	float cf;
	SumN(af, bf, cf);
	cout << "cf = " << cf<<endl;

	vector<int> aa(10,1), bb(10,1), cc;
	vector<float> aaf(10.,3.5), bbf(10.,3.5), ccf;
	//SumN(aa,bb,cc);
	//SumN(aaf,bbf,ccf);
	int ret = find(aa, 1);
	if (ret == aa.size())
		cout << "no found" << endl;
	else
		cout<<"found,index="<<ret<<endl;
#endif
	//产生随机数
#if 0
	srand((unsigned)time(NULL));
	for (int j = 0; j < 20;j++)
	{
		vector<int> outNum;
		ProduceRandNum(1, 100, 50, outNum);
		//outNum = getRandom(1, 100, 50);
		for (int i = 0; i < outNum.size(); i++)
		{
			cout << outNum[i] << " ";
		}
		cout << endl;
	}
#endif
	//模板函数
#if 0
	int i = 23;
	int j = 25;
	float f1 = 23.6;
	float f2 = 25.6;
	cout << mymax(i, j) << endl;
	cout << mymax(f1, f2) << endl;
	//cout << mymin(i, j) << endl;
	//cout << mymin(f1, f2) << endl;
#endif
	//当继承方式为公有继承时，子类的指针可以给基类赋值，这点对于制作一些函数的输入时很有用。
#if 0 
	class A
	{
	};
	class B : public A
	{
	};
	B *b = new B;
	A *a = b;
#endif
	//socket 通讯协议编程
#if 0
	struct sockaddr_in servaddr;
	int listenfd, connfd;
	SOCKET Server_sock = socket(AF_INET, SOCK_STREAM, 0);
	if (Server_sock == -1)
	{
		printf("create socket error: %s(errno: %d)\n", strerror(errno), errno);
		exit(0);
	}
	memset(&servaddr, 0, sizeof(servaddr));
	servaddr.sin_family = AF_INET;
	servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	servaddr.sin_port = htons(6666);
	if( bind(listenfd, (struct sockaddr*)&servaddr, sizeof(servaddr)) == -1){
		printf("bind socket error: %s(errno: %d)\n",strerror(errno),errno);
		exit(0);
	}
#endif
#if 0
	vector<Point3f> points;
	ReadSTLFile("C:/Users/lenovo/Desktop/test.stl", points);
#endif
#endif //CTEST
#pragma endregion C++test


#pragma region 算法测试
#if 0//测试点云正交投影成图片
	for (int k = 0; k < 1;k++)
	{
		clock_t star = clock();
		clock_t mid = clock();
		vector <Point3f> ponts;
		vector <vector<Point3f>> pointsset;
		vector <Mat> rts;
		stringstream path;
		path << "C:/Users/lenovo/Desktop/model/PointCloud.xml";
		XMLReader::readPointsCloud(path.str(), ponts);
		path.str("");
		path << "C:/Users/lenovo/Desktop/model/ObjPoseInCam.xml";
		XMLReader::ReadRT44(path.str(), rts);
		for (int i = 0; i < rts.size(); i++)
		{
			vector <Point3f> _pnts;
			Mat camInObj, R, T;
			Mat projectImg = Mat::zeros(201, 201, CV_8UC3);
			cv::invert(rts[i], camInObj);
			R = camInObj.rowRange(0, 3).colRange(0, 3);
			T = camInObj.rowRange(0, 3).colRange(3, 4);
			int pntInCircleNum = 0;
			circle(projectImg, Point(101, 101), 17, Scalar(255, 0, 0));
			int test = 0;
			for (int j = 0; j < ponts.size(); j++)
			{
				Mat _pnt = (Mat_<double>(3, 1) << ponts[j].x, ponts[j].y, ponts[j].z);
				Mat pntInObj = R*_pnt + T;			
				if (abs(pntInObj.at<double>(0, 0)) < 100 && abs(pntInObj.at<double>(1, 0)) < 100)
				{
					if (abs(pntInObj.at<double>(2, 0)) < 3 || abs(pntInObj.at<double>(2, 0) - 25) < 3)
					{
						projectImg.at<Vec3b>(101 + (int)pntInObj.at<double>(0, 0), 101 + (int)pntInObj.at<double>(1, 0)) = Vec3b(0, 255, 0);
						test++;
					}
				}
				//直接解析点
				if (abs(pntInObj.at<double>(2, 0)) < 1.5  || abs(pntInObj.at<double>(2, 0) - 25) < 1.5)
				{
					int x = pntInObj.at<double>(0, 0);
					int y = pntInObj.at<double>(1, 0);
					if (abs(x) < 20 && abs(y) < 20)
					{
						//计算符合条件点在阈值圆内的数量
						if (sqrt(x*x + y*y) < 15) //16为圆孔的半径值
							pntInCircleNum++;
					}
				}
			}
			cout << "投影点数：" << test << endl;
			mid = clock();
			//cout << "计算圆内点方法：" << mid - star<< " ms" << endl;
			//用直接计算圆内点的方法：
			if (pntInCircleNum < 20)
			{
				cout << "计算圆内点方法：正确识别！";
			}
			else
			{
				cout << "计算圆内点方法：错误识别！";
			}
			//对图像进行闭操作方法：
			Mat outimg;
			Mat ellFitResult;
			Mat kenel = getStructuringElement(MORPH_ELLIPSE, Size2i(5, 5));
			morphologyEx(projectImg, outimg, MORPH_CLOSE, kenel);
			//椭圆拟合
			vector<SH_Ellipse> Fea_circle;
			CoreAlgorithm::DiscernLSCircle(outimg, ellFitResult, Fea_circle, 100, 2);
			for (int i = 0; i < Fea_circle.size(); i++)
			{
				if (Fea_circle[i].brachyaxis <55 && Fea_circle[i].brachyaxis >15)
				{
					if (sqrt(pow(Fea_circle[i].center.x - 101, 2) + pow(Fea_circle[i].center.y - 101, 2)) < 10)
					{
						cout << "图像方法：正确识别" << endl;
						break;
					}
				}
				if (i = Fea_circle.size() - 1)
					cout << "图像方法：错误识别" << endl;
			}

		}
		clock_t end = clock();
		cout << "总共用时：" << end - mid << " ms" << endl;
	}

#endif
#pragma endregion 算法测试
}

#pragma region c++ fun

void testfun2(const vector<float>& input_1, float input_2,vector<float>& out)
{
	//openmp加速
#pragma omp parallel
	{
#pragma  omp for
		for (int i = 0; i < input_1.size(); i++)
		{
			out.push_back(input_1[i] + input_2);
		}
	}
	return;
}


//测试openmp函数
void testfun1( vector<float>& input_1, float input_2)
{
	//openmp加速
#if 0
#pragma omp parallel
	{
#pragma  omp for
		for (int i = 0; i < input_1.size(); i++)
		{
			input_1[i] += input_2;
		}
	}
#endif
	//无加速版
#if 0
	for (int i = 0; i < input_1.size(); i++)
	{
		input_1[i] += input_2;
	}
#endif
	//sse加速(没有内存拷贝的情况)
#if 1
	__m128 b = _mm_set_ps(input_2, input_2, input_2, input_2);
	for (int i = 0; i < input_1.size() - 3; i+=4)
	{
		if (false) //如果是内存对齐
		{
			__m128  *a = reinterpret_cast<__m128*>(&input_1[i]);
			*a = _mm_add_ps(*a, b);
		}
		if (true) //内存没对齐的情况
		{
			__m128  a = _mm_loadu_ps(&input_1[i]);
			a = _mm_add_ps(a, b);
			_mm_store_ps(&input_1[i], a);
		}
		//内存有没有对齐，速度影响不大
	}
#endif
	//sse加速(用内存拷贝的方式),这种方式要比用load方式效率低很多
#if 0
	__m128 b = _mm_set_ps(input_2, input_2, input_2, input_2);
	for (int i = 0; i < input_1.size() - 3; i += 4)
	{
		__m128  a = _mm_set_ps(input_1[i], input_1[i + 1], input_1[i+2], input_1[i+3]);
		__m128 c = _mm_add_ps(a, b);
		_mm_store_ps(&input_1[i], c);
	}
#endif
	return;
}

void outtime(int outTime)
{
	clock_t t1 = clock();
	while (true)
	{
		if (false)
		{
			t1 = clock();
		}
		else
		{
			clock_t t2 = clock();
			if (t2 - t1 > outTime)
			{
				break;
			}
		}
	}
}

DWORD WINAPI ACC(LPVOID pnt)
{
	DWORD re = WaitForSingleObject(mute, INFINITE);
	if (re == WAIT_FAILED)
	{
		printf("wait object faield: %d\n", GetLastError());
		return 1;
	}
	for (int i = 0; i < 10000;i++)
	{
		word++;
		std::cout << "word = " <<word<< endl;
	}
	ReleaseMutex(mute);
	return 1;
}

//template <typename T>
//T SumN(const T &a, const T &b, T &sumValue)
//{
//	sumValue = a + b;
//	return sumValue;
//}
////
////template <typename T>
////T SumN(vector<T> &a, vector<T> &b, vector<T> &sumValue)
////{
////	for (int i = 0; i < a.size(); i++)
////	{
////		sumValue[i] = a[i] + b[i];
////	}
////	return sumValue[0];
////}
//
//template<class T>
//int find(vector<T> &vec, T data)
//{
//	int index = 0;
//	typename vector<T>::iterator iter;
//	for (iter = vec.begin(); iter != vec.end(); iter++)
//	{
//		if (*iter == data)
//			break;
//		else
//			index++;
//	}
//	return index;
//}

DWORD WINAPI muliThreadTest(LPVOID pnt)
{
	while (true)
	{
		cout << "计算中" << endl;
		double rr = 52 * 56 + 45 * 78;
	}
}

bool ReadSTLFile(const char *cfilename, vector<Point3f> &points)
{
	if (cfilename == NULL)
		return false;

	ifstream in(cfilename, ios::in);
	if (!in)
		return false;

	string headStr;
	getline(in, headStr, ' ');
	in.close();

	if (headStr.empty())
		return false;

	if (headStr[0] == 's')
	{
		cout << "ASCII File." << endl;
		ReadASCII(cfilename, points);
	}
	else
	{
		cout << "Binary File." << endl;
		ReadBinary(cfilename, points);
	}
	return true;
}

bool ReadASCII(const char *cfilename, vector<Point3f> &points)
{
	Point3f pnt;

	int i = 0, j = 0, cnt = 0, pCnt = 4;
	char a[100];
	char str[100];
	double x = 0, y = 0, z = 0;

	ifstream in;
	in.open(cfilename, ios::in);
	if (!in)
		return false;
	do
	{
		i = 0;
		cnt = 0;
		in.getline(a, 100, '\n');
		if (a[0] == 'f')
		{
			continue;
		}
		while (a[i] != '\0')
		{
			if (!islower((int)a[i]) && !isupper((int)a[i]) && a[i] != ' ')
				break;
			cnt++;
			i++;
		}

		while (a[cnt] != '\0')
		{
			str[j] = a[cnt];
			cnt++;
			j++;
		}
		str[j] = '\0';
		j = 0;

		if (sscanf(str, "%lf%lf%lf", &x, &y, &z) == 3)
		{
			pnt.x = x;
			pnt.y = y;
			pnt.z = z;
			points.push_back(pnt);
		}
		pCnt++;
	} while (!in.eof());

	//  cout << "******  ACSII FILES　******" << endl;  
	//  for (int i = 0; i < coorX.size();i++)  
	//  {  
	//      cout << coorX[i] << " : " << coorY[i] << " : " << coorZ[i] << endl;  
	//  }  

	cout << points.size() / 3 << " triangles." << endl;
	return true;
}

bool ReadBinary(const char *cfilename, vector<Point3f> &points)
{
	Point3f pnt;

	char str[80];
	ifstream in;

	in.open(cfilename, ios::in | ios::binary);

	if (!in)
		return false;

	in.read(str, 80);

	//number of triangles  
	int unTriangles;
	in.read((char*)&unTriangles, sizeof(int));

	if (unTriangles == 0)
		return false;

	for (int i = 0; i < unTriangles; i++)
	{
		float coorXYZ[12];
		in.read((char*)coorXYZ, 12 * sizeof(float));

		for (int j = 1; j < 4; j++)
		{
			pnt.x = coorXYZ[j * 3];
			pnt.y = coorXYZ[j * 3 + 1];
			pnt.z = coorXYZ[j * 3 + 2];
			
		}

		in.read((char*)coorXYZ, 2);
	}

	in.close();
	cout << points.size() / 3 << " triangles." << endl;
	return true;
}


//template <typename tt> inline tt mymin(tt a, tt b)
//{
//	return a > b ? b : a;
//}

template <typename Td> inline Td const& mymax(Td const& a, Td const& b)
{
	return a < b ? b : a;
}


void testfun(string * &str)
{
	str = &stringGloble;
}

#pragma endregion c++ fun

#pragma region  halcon func

#pragma region opencv fun
void on_trackbar(int Position,void *userData)
{
	Canny(img, outimg, Position, Position * 5, 3, false);
	imshow("canny", outimg);
}

void RansasFitEllipse(const vector<vector <Point>> inContours, RotatedRect outEllipse)
{
	//对轮廓进行分段
}
void ProduceRandNum(int Min, int Max, int num,vector <int> &outNum)
{
	outNum.clear();
	while (outNum.size() < num)
	{
		int ranNum = (rand() % (Max - Min + 1)) + Min;
		bool flag = true;
		for (int i = 0; i < outNum.size();i++)
		{
			if (outNum[i] == ranNum)
			{
				flag = false;
				break;
			}
		}
		if (flag)
		{
			outNum.push_back(ranNum);
		}
	}
}

vector<int> getRandom(int Min, int Max, int total)
{
	srand((int)time(NULL));
	std::vector<int> input = *new std::vector<int>();
	for (int i = 0; i < total; i++) {
		input.push_back(i);
	}
	vector<int> output = *new vector<int>();

	int end = total;
	for (int i = 0; i < total; i++) {
		vector<int>::iterator iter = input.begin();
		int num = (rand() % (Max - Min + 1)) + Min;
		iter = iter + num;
		output.push_back(*iter);
		input.erase(iter);
		end--;
	}

	return output;
}
bool onEllipse(const RotatedRect rotRect, const Point pnt)
{
	float cx = rotRect.center.x;
	float cy = rotRect.center.y;
	float a = rotRect.boundingRect().height;
	float b = rotRect.boundingRect().width;
	float S = rotRect.angle*3.1415926/180;
	int x2 = pnt.x;
	int y2 = pnt.y;
	float result;
	result = pow((cx*cos(S) + cy*sin(S) - x2*cos(S) - y2*sin(S)),2) / (a * a) + pow((cy*cos(S) - cx*sin(S) - y2*cos(S) + x2*sin(S)),2) / (b * b) - 1;
	cout << cos(S)*cos(S) / (a*a) + sin(S)*sin(S) / (b*b) << endl;
	cout << cos(S)*cos(S) / (b*b) + sin(S)*sin(S) / (a*a) << endl;
	if (result < 1 && result > -1)
	{
		cout << "result = " << result << endl;
		return true;
	}
	else
	{
		cout << "result = " << result << endl;
		return false;
	}

}
#pragma endregion opencv fun

