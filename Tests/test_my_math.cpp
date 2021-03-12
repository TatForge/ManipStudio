#include "test_my_math.h"
#if WITH_QTESTS
#include "robo.h"

#include <QtTest/QTest>

using namespace RoboMath;

test_my_math::test_my_math(QObject *parent) : QObject(parent)
{

}

void test_my_math::test_vector_multiple_transpose()
{
    QCOMPARE(Vector({1,2,3,4}).MultiplyTranspose(Vector{3,4,5,6}), 50);
    QCOMPARE(Vector({1,2,3,4}).MultiplyTranspose(Vector{0,0,0,0}), 0);
}

void test_my_math::test_vector_compare()
{
    QCOMPARE(Vector({1,2,3,4}) == Vector({1,2,3,4}), true);
    QCOMPARE(Vector({1,2,3,4}) == Vector({-1,2,3,4}), false);
    QCOMPARE(Vector({1,2,3,4}) == Vector({1,2.001,3.001,4}), false);
    QCOMPARE(Vector({-1.22, -5.009, 123, 54.0009}) == Vector({-1.22, -5.009, 123, 54.0009}), true);
}

void test_my_math::test_matrix_multiple_transpose()
{
    Matrix m {{1,2,3},{5,6,7},{7,8,9},{4,3,2},{7,6,5}};
    Matrix n {{3,4,5,5,4},{7,6,5,4,1},{9,3,2,6,3}};
    Vector v{0.1,0.5,0.9};
    QCOMPARE(m.MultiplyTranspose(m), Matrix({{14,38,50,16,34},{38, 110, 146,52,106},{50,146,194,70,142},{16,52,70,29,56},{34,106,142,56,110}}));
    QCOMPARE(m.MultiplyTranspose(v), Vector({3.8, 9.8, 12.8, 3.7, 8.2}));
}

void test_my_math::test_translation_matrix()
{

}

void test_my_math::test_matrix4x4()
{
    Matrix4x4 m({1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16});
    Matrix4x4 n({6,5,4,3,1,2,3,4,9,8,7,6,5,8,2,6});
    Matrix4x4 m_t({});
    QCOMPARE(m*n, Matrix4x4({55,65,39,53,139,157,103,129,223,249,167,205,307,341,231,281}));
    QCOMPARE(n*m, Matrix4x4({106,124,142,160,90,100,110,120,190,220,250,280,141,162,183,204}));
    QCOMPARE(m(0,0), 1);
    QCOMPARE(m(3,3), 16);
    QCOMPARE(m.MultiplyTranspose(n), Matrix4x4({40, 30, 70, 51, 112, 70, 190, 135, 184, 110, 310, 219, 256, 150, 430, 303}));
    QCOMPARE(m.MultiplyTranspose(m), Matrix4x4({30, 70, 110, 150, 70, 174, 278, 382, 110, 278, 446, 614, 150, 382, 614, 846}));
    QCOMPARE(m.MaxElem(), 2);
    QCOMPARE(n.MaxElem(), 0);
    m.MakeIdentity();
    QCOMPARE(m.MaxElem(), 0);

}
#endif
