#ifndef MY_MATH_H
#define MY_MATH_H

#include <initializer_list>
#include <vector>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <memory>
#include <cstring>
#include <chrono>
#include <assert.h>


#define RAD M_PI/180
#define DEG 180/M_PI

#define DOUBLE_SCALAR
//#define DEBUG_COUT

#ifdef DOUBLE_SCALAR
typedef double scalar;
#else
typedef float scalar;
#endif

typedef unsigned short counter;

using namespace std;


namespace RoboMath
{

    class Quaternion;
    class Vector;
    class Vector3;
    class Matrix;

    class Vector
    {
    public:
        Vector();
        Vector(int size);
        Vector(initializer_list<scalar> il);
        ~Vector();

        int GetLength() const {return length;};
        scalar MaxAbs();
        scalar* GetDataPtr () const {return data;};
        scalar& operator()(counter number);
        Vector& operator=(const Vector &V);
        Vector& operator=(const double &value);
        void  operator*=(const scalar &value);
        Vector  operator-(const Vector &V);
        void VectorCout(const char *ch);
    private:
        int length;
        scalar *data;
        void dataNewMemory(const int &length)
        {
            data = new scalar [length];
        }
    };


    class Vector3
    {
    public:

        Vector3() : x(0), y(0), z(0) {};
        Vector3(const scalar &xn, const scalar &yn, const scalar &zn)
            : x(xn), y(yn), z(zn) {};
        void Rotate(const scalar &theta, const RoboMath::Vector3 &V);
        void Clear();
        scalar  Length()    const {return sqrt(x * x + y * y + z * z);};
        Vector3 Normalize() const;
        Vector3 operator= (const Vector &V);
        Vector3 operator= (const Vector3 &V);
        Vector3 operator- (const Vector3 &V) const;
        Vector3 operator+ (const Vector3 &V) const;
        Vector3 operator* (const Vector3 &V) const;
        Vector3 &operator+= (const Vector3 &V);
        Vector3 &operator* (const scalar &value);
    public:
        scalar x;
        scalar y;
        scalar z;
    };


    const Vector3 unit_x(1.0, 0.0, 0.0);
    const Vector3 unit_y(0.0, 1.0, 0.0);
    const Vector3 unit_z(0.0, 0.0, 1.0);
    const Vector3 unit_mz(0.0, 0.0, -1.0);

    class Matrix
    {
    public:
        Matrix();
        Matrix(const int &rows, const int &cols);
        Matrix(const Matrix& source);
        Matrix(const initializer_list<initializer_list<scalar>> &il);
        Matrix(const initializer_list<scalar> &il);
        ~Matrix()
        {
            if(data != nullptr)
                delete[] data;
        }
        scalar &at(const int &row, const int &col) const;

        Matrix Multuply(const Matrix &matrix1, const Matrix &matrix2) const;
        size_t Size() const;
        void Transpose();
        void AddDiagonal(const scalar &d);
        void SetSubmatrix(const Matrix &source, const int col, const int row);
        void SetColumn(const Vector &v, const int ColNumber);
        void SetColumn(const Vector3 &v, const int col);
        void SetSubColumn(const Vector3 &v, const int ColNumber, const int RowNumber);
        void SolveGauss();               //Решение СЛАУ методом Гаусса (приведение к ступенчатому виду)
        void GetTheRoots(Vector &V) const;     //Получение корней уравнения
        Matrix operator*(const Matrix &matrix2) const;
        Vector operator*(const Vector &vector2) const;
        Matrix operator=(const Matrix &matrix1);

        scalar& operator()(const int &row, const int &col);
        int GetColsCount() const { return cols; }
        int GetRowsCount() const { return rows; }
        scalar* GetDataPtr() const {return data;};
        void MatrixCout() const;


    private:
        int rows;
        int cols;
        scalar *data;
        void DataNewMemory()
        {
            data = new scalar[rows*cols];
        }
        void DataNewMemory(const int &rows, const int &cols)
        {
            data = new scalar[rows*cols];
    #ifdef DEBUG_COUT
            cout << "Data matrix was initialized with rows = "
                 << rows << " end cols = " << cols << endl;
            cout << endl;
    #endif
        }
        int index(const int &row, const int &col) const { return row * cols + col;}
    };
    class Quaternion
    {
    public:
        Quaternion() : x(0), y(0), z(0), w(1){};
        Quaternion(const scalar &x_in, const scalar &y_in, const scalar &z_in, const scalar &w_in);
        Quaternion(const RoboMath::Vector3 &coordXYZ, const scalar &angle);
        void Create(const RoboMath::Vector3 &rotate_vector, const scalar &rotate_angle);
        void EulerToQuat(const Vector3 &angles);
        Vector3 GetEulerAngles() const;
        void Invert();
        scalar GetLength() const;
        Vector3 GetAxis() const;
        long double GetAngle() const;
        Quaternion operator* (const Quaternion &q) const;
        Quaternion operator* (const Vector3 &v) const;
        void Scale(const scalar &s);
        void Normalize();
        Vector3 RotateVector(const Vector3 &v);
    private:
        scalar x;
        scalar y;
        scalar z;
        scalar w;
    };
}

class checkTime
{
public:
    checkTime()
    {
        start = std::chrono::high_resolution_clock::now();
        checked = false;
    }
    ~checkTime()
    {
        if (!checked)
        {
            stop = std::chrono::high_resolution_clock::now();
            d = stop - start;
            cout << "duration = " << d.count() << endl << " microsec";
        }
    }
    std::chrono::duration<double> duration()
    {
        stop = std::chrono::high_resolution_clock::now();
        d = stop - start;
        checked = true;
        return d;
    }

private:
    std::chrono::duration<double> d;
    std::chrono::_V2::system_clock::time_point start;
    std::chrono::_V2::system_clock::time_point stop;
    bool checked;
};

class AllMath
{
    template<typename T> static inline T min(T t1, T t2) {return (t1 < t2 ? t1 : t2);};
    template<typename T> static inline T max(T t1, T t2) {return (t1 > t2 ? t1 : t2);};
};

#endif // MY_MATH_H
