#include "test_my_math.h"
#include "my_math.h"

#include <QTest>

using namespace RoboMath;

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
    QCOMPARE(m.MultiplyTranspose(m), Matrix({{14,38,50,16,34},{38, 110, 146,52,106},{50,146,194,70,142},{16,52,70,29,56},{34,106,142,56,110}}));
    QCOMPARE(m.MultiplyTranspose(Vector({0.1,0.5,0.9})), Vector({3.8, 9.8, 12.8, 3.7, 8.2}));
}
