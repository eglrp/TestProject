#pragma once
#include <memory>
class test
{
public:
	test() {};
	~test() { std::cout << "dispose test" << std::endl; };
	static shared_ptr<test> init()
	{
		static shared_ptr<test> df(new test()); //���ع���ָ��
		return df;
	}
	int memberd = 1;
};

