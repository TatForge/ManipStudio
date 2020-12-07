#include <QtTest>
#include "my_math.h"

// add necessary includes here
using namespace RoboMath;
Q_DECLARE_METATYPE(Vector);
class TestRobot : public QObject
{
    Q_OBJECT

public:
    TestRobot();
    ~TestRobot();

private slots:
    void test_vector();
    void test_vector_data();

};

TestRobot::TestRobot()
{

}

TestRobot::~TestRobot()
{

}

void TestRobot::test_vector()
{
    QFETCH(Vector, a);
    QFETCH(Vector, b);
    QFETCH(scalar, result);
    Vector av{1,2,3,4};
    Vector bv{3,4,5,6};

    QCOMPARE(av.MultiplyTranspose(bv), 50);
}
void TestRobot::test_vector_data()
{
//    QTest::addColumn<Vector>("a");
//    QTest::addColumn<Vector>("b");
//    QTest::addColumn<scalar>("result");

//    QTest::newRow("Multiple transpose") << Vector{1,2,3,4} << Vector{3,4,5,6} << scalar(50);
//    QTest::newRow("Multiple transpose") << Vector{1,2,3,4} << Vector{3,4,5,6} << scalar(50);

}

QTEST_APPLESS_MAIN(TestRobot)

//#include "tst_testrobot.moc"
