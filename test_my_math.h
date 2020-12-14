#ifndef TEST_MY_MATH_H
#define TEST_MY_MATH_H

//#define WITH_TESTS

#ifdef WITH_TESTS
#include <QObject>


class test_my_math : public QObject
{
    Q_OBJECT
public:
//    explicit test_my_math(QObject *parent = nullptr);
//    ~test_my_math();

private slots:
    void test_vector_multiple_transpose();
    void test_vector_compare();
    void test_matrix_multiple_transpose();
    void test_translation_matrix();

};
#endif // WITH_TESTS

#endif // TEST_MY_MATH_H
