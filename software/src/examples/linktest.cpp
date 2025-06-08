
#include <cmath>
#include <cstdio>

class TestClass
{
	public:
	TestClass(int n);
	virtual ~TestClass();
	virtual int VirtualMethod(int n);
	double Method(int n);
	private:
	int member1;
	double member2;
};

TestClass::TestClass(int n)
{
	member1 = n;
	member2 = n*0.001;
}

TestClass::~TestClass()
{
	member1 = 0;
	member2 = 0.0;
}

int TestClass::VirtualMethod(int n)
{
	member2 = sqrt(n*4.0);
	member1 = n;
	return n;
}

double TestClass::Method(int n)
{
	return member2 + n;
}

TestClass tc(0);

int main(int argc, const char *argv[])
{
	printf("Virtual = %d\n", tc.VirtualMethod(1));

	return 0;
}

